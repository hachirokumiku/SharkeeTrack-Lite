#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <MadgwickAHRS.h>
#include <ArduinoOSC.h>
#include <math.h>
#include <vector>

// ===== CONFIG =====
const char* ssid = "YourSSID";
const char* password = "YourPASSWORD";
const unsigned int udpPort = 4210;
const char* oscHostIP = "192.168.1.100";
const unsigned int oscHostPort = 9000;

const int TOTAL_TRACKERS = 10;
const int VIRTUAL_BONES = 20;
const float ALPHA_IMU = 0.05f;
const float ALPHA_RSSI = 0.02f;
const float TX_POWER = -40.0f;
const int SMOOTH_SAMPLES = 20;

// Battery voltage divider
const float R_TOP = 180000.0;      // 180kΩ to battery +
const float R_BOTTOM = 47000.0;    // 47kΩ to GND
const float ADC_MAX = 1023.0;
const float ESP_VREF = 1.0;        // max voltage at A0

// ===== GLOBALS =====
WiFiUDP udp;
BMI270 imu;
Madgwick filter;
OSCClass osc;
String myID;

// IMU + smoothing
struct Vec3 { float x,y,z; };
struct Quat { float w,x,y,z; };
Vec3 rawAccel, rawGyro;
Vec3 smoothedAccel={0,0,0};
Vec3 smoothedGyro={0,0,0};
Quat smoothedQuat={1,0,0,0};
std::vector<Vec3> accelBuffer(SMOOTH_SAMPLES);
std::vector<Vec3> gyroBuffer(SMOOTH_SAMPLES);
std::vector<Quat> quatBuffer(SMOOTH_SAMPLES);
int bufIndex=0;

// Tracker assignment
int trackerAssignment[TOTAL_TRACKERS]; 
bool assigned[TOTAL_TRACKERS]={false};

// Virtual bones
struct Bone{ Vec3 position; Quat rotation; };
Bone virtualBones[VIRTUAL_BONES]; 

// Inter-tracker RSSI/distance matrix
float rssiMatrix[TOTAL_TRACKERS][TOTAL_TRACKERS];
int rssiCount[TOTAL_TRACKERS][TOTAL_TRACKERS];

// Battery
float batteryVoltage = 0;
float batterySmoothed = 0;

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

    for(int i=0;i<TOTAL_TRACKERS;i++){ 
        trackerAssignment[i]=-1; 
        for(int j=0;j<TOTAL_TRACKERS;j++){ rssiMatrix[i][j]=1.0; rssiCount[i][j]=0; } 
    }
}

// ===== LOOP =====
void loop(){
    // --- Read IMU ---
    if(imu.getSensorData()==BMI2_OK){
        rawAccel={imu.data.accelX/16384.0f, imu.data.accelY/16384.0f, imu.data.accelZ/16384.0f};
        rawGyro={imu.data.gyroX/131.0f, imu.data.gyroY/131.0f, imu.data.gyroZ/131.0f};
        filter.updateIMU(rawGyro.x, rawGyro.y, rawGyro.z, rawAccel.x, rawAccel.y, rawAccel.z);
        Quat currentQuat={filter.getQuaternionW(), filter.getQuaternionX(), filter.getQuaternionY(), filter.getQuaternionZ()};

        // --- EMA smoothing ---
        float gyroMag = sqrt(rawGyro.x*rawGyro.x + rawGyro.y*rawGyro.y + rawGyro.z*rawGyro.z);
        float adaptiveAlpha = constrain(ALPHA_IMU + gyroMag/200.0f, ALPHA_IMU, 0.25f);
        smoothedAccel = vecLerp(smoothedAccel, rawAccel, adaptiveAlpha);
        smoothedGyro = vecLerp(smoothedGyro, rawGyro, adaptiveAlpha);
        smoothedQuat = quatSlerp(smoothedQuat, currentQuat, adaptiveAlpha);
    }

    // --- Battery read ---
    int adc = analogRead(A0);
    float vA0 = ((float)adc / ADC_MAX) * ESP_VREF;
    batteryVoltage = vA0 * (R_TOP + R_BOTTOM) / R_BOTTOM;
    batterySmoothed = batterySmoothed*0.8 + batteryVoltage*0.2; // smooth
    batteryVoltage = batterySmoothed;

    // --- Auto assignment ---
    static bool chestBooted=false, hipBooted=false;
    if(!chestBooted){ chestBooted=true; trackerAssignment[0]=0; Serial.println("Chest booted"); }
    if(!hipBooted){ hipBooted=true; trackerAssignment[1]=1; Serial.println("Hip booted"); }
    for(int i=2;i<TOTAL_TRACKERS;i++){ if(!assigned[i]){ trackerAssignment[i]=i; assigned[i]=true; Serial.printf("Tracker %d assigned\n", i); break; } }

    // --- Inter-tracker distance stacking ---
    for(int i=0;i<TOTAL_TRACKERS;i++){
        Vec3 correctionTotal={0,0,0}; float weightSum=0.0f;
        for(int j=0;j<TOTAL_TRACKERS;j++){
            if(i==j) continue;
            float weight = 1.0 / max(0.01f, rssiMatrix[i][j]); 
            Vec3 delta = {smoothedAccel.x*weight, smoothedAccel.y*weight, smoothedAccel.z*weight};
            correctionTotal.x += delta.x; correctionTotal.y += delta.y; correctionTotal.z += delta.z;
            weightSum += weight;
        }
        if(weightSum>0){
            correctionTotal.x /= weightSum; correctionTotal.y /= weightSum; correctionTotal.z /= weightSum;
            smoothedAccel = vecLerp(smoothedAccel, correctionTotal, ALPHA_RSSI);
        }
    }

    // --- Update virtual bones ---
    for(int i=0;i<TOTAL_TRACKERS;i++){
        int boneIndex=i*2;
        if(boneIndex<VIRTUAL_BONES){
            virtualBones[boneIndex].position = vecLerp(virtualBones[boneIndex].position, smoothedAccel, ALPHA_IMU);
            virtualBones[boneIndex].rotation = quatSlerp(virtualBones[boneIndex].rotation, smoothedQuat, ALPHA_IMU);
        }
    }

    // --- Send UDP + OSC ---
    char outBuffer[512];
    snprintf(outBuffer,sizeof(outBuffer),
        "%s,AX%.3f,AY%.3f,AZ%.3f,GX%.3f,GY%.3f,GZ%.3f,BATT%.3f",
        myID.c_str(),
        smoothedAccel.x, smoothedAccel.y, smoothedAccel.z,
        smoothedGyro.x, smoothedGyro.y, smoothedGyro.z,
        batteryVoltage);
    udp.beginPacket("255.255.255.255",udpPort);
    udp.write(outBuffer);
    udp.endPacket();

    OSCMessage msg("/tracker");
    msg.add(myID.c_str());
    msg.add(smoothedAccel.x); msg.add(smoothedAccel.y); msg.add(smoothedAccel.z);
    msg.add(smoothedQuat.w); msg.add(smoothedQuat.x); msg.add(smoothedQuat.y); msg.add(smoothedQuat.z);
    msg.add(batteryVoltage);
    osc.send(msg, oscHostIP, oscHostPort); msg.empty();

    delay(10); // 100Hz loop
}
