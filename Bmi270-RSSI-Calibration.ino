#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <ArduinoOSC.h>
#include <vector>
#include <math.h>

// ===== CONFIG =====
const char* ssid = "YourSSID";
const char* password = "YourPASSWORD";
const unsigned int udpPort = 4210;
const char* oscHostIP = "192.168.1.100";
const unsigned int oscHostPort = 9000;

const int TOTAL_TRACKERS = 10;
const int SMOOTH_SAMPLES = 20;
const float ALPHA_IMU = 0.05;    // IMU smoothing
const float ALPHA_RSSI = 0.02;   // RSSI smoothing
const float TX_POWER = -40.0;    // RSSI @ 1m
const unsigned long TPoseDelay = 2000; // 2s

// ===== GLOBALS =====
WiFiUDP udp;
BMI270 imu;
Madgwick filter;
OSCClass osc;
String myID;

struct Vec3 { float x,y,z; };
struct Quat { float w,x,y,z; };

// IMU buffers
Vec3 rawAccel, rawGyro;
Vec3 smoothedAccel={0,0,0};
Vec3 smoothedGyro={0,0,0};
Quat smoothedQuat={1,0,0,0};
std::vector<Vec3> accelBuffer(SMOOTH_SAMPLES);
std::vector<Vec3> gyroBuffer(SMOOTH_SAMPLES);
std::vector<Quat> quatBuffer(SMOOTH_SAMPLES);
int bufIndex=0;

// Tracker assignment
int trackerAssignment[TOTAL_TRACKERS]; // maps physical tracker -> body part
bool assigned[TOTAL_TRACKERS]={false};
bool chestBooted=false;
bool hipBooted=false;

// Virtual bones (20-point humanoid)
struct Bone{ Vec3 position; Quat rotation; };
Bone virtualBones[20]; 

// T-pose
bool tPoseCalibrated=false;
unsigned long tPoseStart=0;

// Inter-tracker RSSI
float rssiMatrix[TOTAL_TRACKERS][TOTAL_TRACKERS];
int rssiCount[TOTAL_TRACKERS][TOTAL_TRACKERS];

// ===== UTILITIES =====
float rssiToDistance(float rssi){ return pow(10.0,(TX_POWER - rssi)/20.0); }

Vec3 vecLerp(const Vec3& a,const Vec3& b,float alpha){
    return {a.x*(1-alpha)+b.x*alpha, a.y*(1-alpha)+b.y*alpha, a.z*(1-alpha)+b.z*alpha};
}

Quat quatSlerp(const Quat& a,const Quat& b,float alpha){
    float dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    float blendI = 1 - alpha;
    Quat q;
    if(dot < 0){
        q.w = blendI*a.w + alpha*-b.w;
        q.x = blendI*a.x + alpha*-b.x;
        q.y = blendI*a.y + alpha*-b.y;
        q.z = blendI*a.z + alpha*-b.z;
    } else{
        q.w = blendI*a.w + alpha*b.w;
        q.x = blendI*a.x + alpha*b.x;
        q.y = blendI*a.y + alpha*b.y;
        q.z = blendI*a.z + alpha*b.z;
    }
    float mag = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w/=mag; q.x/=mag; q.y/=mag; q.z/=mag;
    return q;
}

Vec3 smoothBuffer(const std::vector<Vec3>& buffer){
    Vec3 avg={0,0,0};
    for(auto& v:buffer){ avg.x+=v.x; avg.y+=v.y; avg.z+=v.z; }
    avg.x/=buffer.size(); avg.y/=buffer.size(); avg.z/=buffer.size();
    return avg;
}

Quat smoothQuatBuffer(const std::vector<Quat>& buffer){
    Quat avg={0,0,0,0};
    for(auto& q:buffer){ avg.w+=q.w; avg.x+=q.x; avg.y+=q.y; avg.z+=q.z; }
    float mag=sqrt(avg.w*avg.w + avg.x*avg.x + avg.y*avg.y + avg.z*avg.z);
    avg.w/=mag; avg.x/=mag; avg.y/=mag; avg.z/=mag;
    return avg;
}

// ===== SETUP =====
void setup(){
    Serial.begin(115200); delay(2000);
    WiFi.mode(WIFI_STA); WiFi.begin(ssid,password);
    Serial.print("Connecting"); while(WiFi.status()!=WL_CONNECTED){delay(500); Serial.print(".");}
    Serial.println("\nWiFi connected!");
    udp.begin(udpPort); osc.begin(oscHostPort);
    myID = String(WiFi.macAddress());
    Serial.printf("Tracker ID: %s\n",myID.c_str());
    Wire.begin(D2,D1);
    if(imu.begin()!=BMI2_OK){Serial.println("BMI270 init failed!"); while(1) delay(100);}
    filter.begin(50); // 50Hz
    for(int i=0;i<SMOOTH_SAMPLES;i++){ accelBuffer[i]={0,0,0}; gyroBuffer[i]={0,0,0}; quatBuffer[i]={1,0,0,0}; }
    for(int i=0;i<TOTAL_TRACKERS;i++){ trackerAssignment[i]=-1; for(int j=0;j<TOTAL_TRACKERS;j++){ rssiMatrix[i][j]=1.0; rssiCount[i][j]=0; } }
}

// ===== LOOP =====
void loop(){
    // --- IMU read ---
    if(imu.getSensorData()==BMI2_OK){
        rawAccel={imu.data.accelX/16384.0f, imu.data.accelY/16384.0f, imu.data.accelZ/16384.0f};
        rawGyro={imu.data.gyroX/131.0f, imu.data.gyroY/131.0f, imu.data.gyroZ/131.0f};
        accelBuffer[bufIndex]=rawAccel; gyroBuffer[bufIndex]=rawGyro;
        filter.updateIMU(rawGyro.x, rawGyro.y, rawGyro.z, rawAccel.x, rawAccel.y, rawAccel.z);
        Quat q={filter.getQuaternionW(), filter.getQuaternionX(), filter.getQuaternionY(), filter.getQuaternionZ()};
        quatBuffer[bufIndex]=q;
        bufIndex=(bufIndex+1)%SMOOTH_SAMPLES;
        smoothedAccel=smoothBuffer(accelBuffer);
        smoothedGyro=smoothBuffer(gyroBuffer);
        smoothedQuat=smoothQuatBuffer(quatBuffer);
    }

    // --- Assign trackers automatically ---
    if(!chestBooted){ chestBooted=true; trackerAssignment[0]=0; Serial.println("Chest booted"); }
    if(!hipBooted){ hipBooted=true; trackerAssignment[1]=1; Serial.println("Hip booted"); }
    for(int i=2;i<TOTAL_TRACKERS;i++){ if(!assigned[i]){ trackerAssignment[i]=i; assigned[i]=true; Serial.printf("Tracker %d assigned\n", i); break; } }

    // --- Wait for all trackers to be ready ---
    int powered=0; for(int i=0;i<TOTAL_TRACKERS;i++) if(trackerAssignment[i]>=0) powered++;
    if(powered<TOTAL_TRACKERS){ delay(10); return; }

    // --- Automatic T-pose detection ---
    static unsigned long stableStart=0;
    static Vec3 lastAccel=smoothedAccel;
    float delta = sqrt(pow(smoothedAccel.x-lastAccel.x,2)+pow(smoothedAccel.y-lastAccel.y,2)+pow(smoothedAccel.z-lastAccel.z,2));
    if(delta<0.01){ // stable
        if(stableStart==0) stableStart=millis();
    } else { stableStart=0; }
    lastAccel=smoothedAccel;

    if(!tPoseCalibrated && stableStart>0 && millis()-stableStart>=TPoseDelay){
        // take T-pose snapshot
        for(int i=0;i<20;i++){ virtualBones[i].position=smoothedAccel; virtualBones[i].rotation=smoothedQuat; }
        tPoseCalibrated=true;
        Serial.println("Automatic T-pose snapshot taken, virtual bones initialized.");
    }

    // --- Update virtual bones ---
    if(tPoseCalibrated){
        for(int i=0;i<TOTAL_TRACKERS;i++){
            int boneIndex=i*2; // simple mapping
            virtualBones[boneIndex].position = vecLerp(virtualBones[boneIndex].position, smoothedAccel, ALPHA_IMU);
            virtualBones[boneIndex].rotation = quatSlerp(virtualBones[boneIndex].rotation, smoothedQuat, ALPHA_IMU);
        }
    }

    // --- Send UDP + OSC ---
    char outBuffer[512];
    snprintf(outBuffer,sizeof(outBuffer),
        "%s,AX%.3f,AY%.3f,AZ%.3f,GX%.3f,GY%.3f,GZ%.3f",
        myID.c_str(),
        smoothedAccel.x, smoothedAccel.y, smoothedAccel.z,
        smoothedGyro.x, smoothedGyro.y, smoothedGyro.z);
    udp.beginPacket("255.255.255.255", udpPort); udp.write(outBuffer); udp.endPacket();

    OSCMessage msg("/avatar/pose");
    msg.add(myID.c_str());
    for(int i=0;i<20;i++){
        msg.add(virtualBones[i].position.x); msg.add(virtualBones[i].position.y); msg.add(virtualBones[i].position.z);
        msg.add(virtualBones[i].rotation.w); msg.add(virtualBones[i].rotation.x); msg.add(virtualBones[i].rotation.y); msg.add(virtualBones[i].rotation.z);
    }
    osc.send(msg, oscHostIP, oscHostPort);
    msg.empty();

    delay(10);
}
