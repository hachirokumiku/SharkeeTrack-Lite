#include <ESP8266WiFi.h> #include <WiFiUdp.h> #include <SparkFun_BMI270_Arduino_Library.h> #include <Wire.h> #include <MadgwickAHRS.h> #include <ArduinoOSC.h> #include <vector>

// ===== CONFIG ===== const char* ssid = "YourSSID"; const char* password = "YourPASSWORD"; const unsigned int udpPort = 4210; const char* oscHostIP = "192.168.1.100"; const unsigned int oscHostPort = 9000; const int TAP_THRESHOLD = 1500; const int SMOOTH_SAMPLES = 20; const float TX_POWER = -40.0; const float PEER_ALPHA = 0.05; const float SELF_CAL_ALPHA = 0.02; const int TOTAL_TRACKERS = 10; const int MIN_CAL_TRACKERS = 3; // chest, hip, router

// ===== GLOBALS ===== WiFiUDP udp; BMI270 imu; Madgwick filter; OSCClass osc; char incomingPacket[512]; String myID;

struct Vec3 { float x,y,z; }; struct Quat { float w,x,y,z; };

Vec3 rawAccel, rawGyro; Vec3 smoothedAccel={0,0,0}; Vec3 smoothedGyro={0,0,0}; Quat smoothedQuat={1,0,0,0}; std::vector<Vec3> accelBuffer(SMOOTH_SAMPLES); std::vector<Vec3> gyroBuffer(SMOOTH_SAMPLES); std::vector<Quat> quatBuffer(SMOOTH_SAMPLES); int bufIndex=0;

Vec3 offset={0,0,0}; Vec3 northVector={0,0,0}; Vec3 peerVector={0,0,0};

bool tPoseCalibrated=false; unsigned long tPoseStart=0; const unsigned long TPoseDelay=2000; int poweredTrackers=0;

int tapCount=0; bool tapTriggered=false; unsigned long tapTime=0;

// ===== UTILITIES ===== float rssiToDistance(float rssi){ return pow(10.0,(TX_POWER-rssi)/20.0); } Vec3 smoothBuffer(const std::vector<Vec3>& buffer){ Vec3 avg={0,0,0}; for(auto& v:buffer){avg.x+=v.x; avg.y+=v.y; avg.z+=v.z;} avg.x/=buffer.size();avg.y/=buffer.size();avg.z/=buffer.size(); return avg; } Quat smoothQuatBuffer(const std::vector<Quat>& buffer){ Quat avg={0,0,0,0}; for(auto& q:buffer){avg.w+=q.w;avg.x+=q.x;avg.y+=q.y;avg.z+=q.z;} float mag=sqrt(avg.wavg.w+avg.xavg.x+avg.yavg.y+avg.zavg.z); avg.w/=mag;avg.x/=mag;avg.y/=mag;avg.z/=mag; return avg; } bool detectTap(const Vec3& accel){ return sqrt(accel.xaccel.x+accel.yaccel.y+accel.zaccel.z)>TAP_THRESHOLD; } Vec3 vecLerp(const Vec3& a,const Vec3& b,float alpha){ return {a.x(1-alpha)+b.xalpha, a.y(1-alpha)+b.yalpha, a.z(1-alpha)+b.zalpha}; } Quat quatLerp(const Quat& a,const Quat& b,float alpha){ Quat q={a.w(1-alpha)+b.walpha,a.x(1-alpha)+b.xalpha,a.y(1-alpha)+b.yalpha,a.z(1-alpha)+b.zalpha}; float mag=sqrt(q.wq.w+q.xq.x+q.yq.y+q.z*q.z); q.w/=mag;q.x/=mag;q.y/=mag;q.z/=mag; return q; }

void setup(){ Serial.begin(115200); delay(2000); WiFi.mode(WIFI_STA); WiFi.begin(ssid,password); Serial.print("Connecting"); while(WiFi.status()!=WL_CONNECTED){delay(500);Serial.print(".");} Serial.println("\nWiFi connected!"); udp.begin(udpPort); osc.begin(oscHostPort); myID=String(WiFi.macAddress()); Serial.printf("Tracker ID: %s\n",myID.c_str()); Wire.begin(D2,D1); if(imu.begin()!=BMI2_OK){Serial.println("BMI270 init failed!"); while(1) delay(100);} Serial.println("BMI270 ready"); filter.begin(50); // sample rate 50Hz for(int i=0;i<SMOOTH_SAMPLES;i++){accelBuffer[i]={0,0,0};gyroBuffer[i]={0,0,0}; quatBuffer[i]={1,0,0,0};} }

void loop(){ if(imu.getSensorData()==BMI2_OK){ rawAccel={imu.data.accelX/16384.0f,imu.data.accelY/16384.0f,imu.data.accelZ/16384.0f}; rawGyro={imu.data.gyroX/131.0f,imu.data.gyroY/131.0f,imu.data.gyroZ/131.0f}; accelBuffer[bufIndex]=rawAccel; gyroBuffer[bufIndex]=rawGyro; filter.updateIMU(rawGyro.x,rawGyro.y,rawGyro.z,rawAccel.x,rawAccel.y,rawAccel.z); Quat q={filter.getQuaternionW(),filter.getQuaternionX(),filter.getQuaternionY(),filter.getQuaternionZ()}; quatBuffer[bufIndex]=q; bufIndex=(bufIndex+1)%SMOOTH_SAMPLES; smoothedAccel=smoothBuffer(accelBuffer); smoothedGyro=smoothBuffer(gyroBuffer); smoothedQuat=smoothQuatBuffer(quatBuffer); }

// Wait until minimum trackers for initial calibration int packetSize=udp.parsePacket(); if(packetSize){int len=udp.read(incomingPacket,512); if(len>0) incomingPacket[len]='\0'; poweredTrackers++;} if(poweredTrackers<MIN_CAL_TRACKERS){ delay(10); return; } // wait for chest, hip, router

// Initial T-pose calibration once minimum trackers are powered if(!tPoseCalibrated && detectTap(smoothedAccel) && !tapTriggered){tapTriggered=true; tapTime=millis(); tapCount++;} if(tapTriggered && millis()-tapTime>=200){tapTriggered=false; if(tapCount>=3){tPoseStart=millis(); tapCount=0;}} if(!tPoseCalibrated && tPoseStart>0 && millis()-tPoseStart>=TPoseDelay){offset.x=-smoothedAccel.x; offset.y=-smoothedAccel.y; offset.z=-smoothedAccel.z; tPoseCalibrated=true;}

// Self-calibration and stacking as more trackers power on Vec3 correctedAccel = smoothedAccel; correctedAccel = vecLerp(correctedAccel, peerVector, SELF_CAL_ALPHA); correctedAccel = vecLerp(correctedAccel, northVector, SELF_CAL_ALPHA); smoothedAccel = correctedAccel;

// Send position + rotation + north + peer vector over UDP char outBuffer[512]; snprintf(outBuffer,sizeof(outBuffer),"%s,AX%.3f,AY%.3f,AZ%.3f,GX%.3f,GY%.3f,GZ%.3f,CX%.3f,CY%.3f,CZ%.3f,NX%.3f,NY%.3f,NZ%.3f,PX%.3f,PY%.3f,PZ%.3f", myID.c_str(), smoothedAccel.x+offset.x, smoothedAccel.y+offset.y, smoothedAccel.z+offset.z, smoothedGyro.x, smoothedGyro.y, smoothedGyro.z, offset.x, offset.y, offset.z, northVector.x,northVector.y,northVector.z, peerVector.x,peerVector.y,peerVector.z); udp.beginPacket("255.255.255.255",udpPort); udp.write(outBuffer); udp.endPacket();

OSCMessage msg("/tracker"); msg.add(myID.c_str()); msg.add(smoothedAccel.x+offset.x); msg.add(smoothedAccel.y+offset.y); msg.add(smoothedAccel.z+offset.z); msg.add(smoothedQuat.w); msg.add(smoothedQuat.x); msg.add(smoothedQuat.y); msg.add(smoothedQuat.z); msg.add(northVector.x); msg.add(northVector.y); msg.add(northVector.z); msg.add(peerVector.x); msg.add(peerVector.y); msg.add(peerVector.z); osc.send(msg, oscHostIP, oscHostPort); msg.empty();

delay(10); }

