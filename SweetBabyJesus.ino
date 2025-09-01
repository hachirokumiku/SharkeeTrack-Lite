#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <MadgwickAHRS.h>
#include <ArduinoOSCWiFi.h>
#include <vector>

// ===== CONFIG =====
const char* ssid = "wxyz";
const char* password = "wxyz";
const unsigned int udpPort = 4210;
const char* oscHostIP = "10.0.0.216";
const unsigned int oscHostPort = 9000;
const int TOTAL_TRACKERS = 10;
const int SMOOTH_SAMPLES = 20;
const unsigned long SLOT_DURATION_US = 1000; // 1ms TDMA slot

// ===== GLOBALS =====
WiFiUDP udp;
OSCClass osc;
BMI270 imu;
Madgwick filter;

struct Vec3 { float x,y,z; };
struct Quat { float w,x,y,z; };
struct Peer {
  String id;
  Vec3 accel;
  Quat quat;
  Quat virtualBones[10];
  float battery;
};

Vec3 rawAccel, rawGyro;
Vec3 accelBuffer[SMOOTH_SAMPLES];
Vec3 gyroBuffer[SMOOTH_SAMPLES];
Quat quatBuffer[SMOOTH_SAMPLES];
int bufIndex = 0;

Vec3 smoothedAccel={0,0,0};
Vec3 smoothedGyro={0,0,0};
Quat smoothedQuat={1,0,0,0};
Quat virtualBoneQuat[10];
Peer peers[TOTAL_TRACKERS];

float batteryVoltage = 0;
int mySlot = 0;

// ===== FEC + CRC =====
struct FECBlock { uint8_t data[128]; uint8_t parity; };
void encodeFEC(FECBlock &b){ b.parity=0; for(int i=0;i<128;i++) b.parity^=b.data[i]; }
bool decodeFEC(FECBlock &b){ uint8_t p=0; for(int i=0;i<128;i++) p^=b.data[i]; return p==b.parity; }

// ===== UTILS =====
Vec3 vecLerp(const Vec3 &a,const Vec3 &b,float alpha){
  return {a.x*(1-alpha)+b.x*alpha, a.y*(1-alpha)+b.y*alpha, a.z*(1-alpha)+b.z*alpha};
}
Quat quatLerp(const Quat &a,const Quat &b,float alpha){
  Quat q={a.w*(1-alpha)+b.w*alpha, a.x*(1-alpha)+b.x*alpha, a.y*(1-alpha)+b.y*alpha, a.z*(1-alpha)+b.z*alpha};
  float mag = sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
  q.w/=mag; q.x/=mag; q.y/=mag; q.z/=mag;
  return q;
}

// ===== SETUP =====
void setup(){
  WiFi.mode(WIFI_STA); WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED) delay(10);
  udp.begin(udpPort); osc.begin(oscHostPort);
  Wire.begin(D2,D1);

  if(imu.begin()!=BMI2_OK) while(1) delay(100);
  filter.begin(200); // max rate
  for(int i=0;i<SMOOTH_SAMPLES;i++){
    accelBuffer[i]={0,0,0};
    gyroBuffer[i]={0,0,0};
    quatBuffer[i]={1,0,0,0};
  }

  // Assign a TDMA slot based on MAC address
  String mac = WiFi.macAddress();
  mySlot = mac[mac.length()-1] % TOTAL_TRACKERS; // simple hash
}

// ===== BATTERY =====
void readBattery(){ batteryVoltage = analogRead(A0)/1024.0*3.3; }

// ===== IMU UPDATE =====
void updateIMU(){
  if(imu.getSensorData()==BMI2_OK){
    rawAccel={imu.data.accelX/16384.0f, imu.data.accelY/16384.0f, imu.data.accelZ/16384.0f};
    rawGyro={imu.data.gyroX/131.0f, imu.data.gyroY/131.0f, imu.data.gyroZ/131.0f};

    accelBuffer[bufIndex]=rawAccel; gyroBuffer[bufIndex]=rawGyro;
    filter.updateIMU(rawGyro.x,rawGyro.y,rawGyro.z, rawAccel.x,rawAccel.y,rawAccel.z);
    quatBuffer[bufIndex]={filter.getQuaternionW(), filter.getQuaternionX(), filter.getQuaternionY(), filter.getQuaternionZ()};
    bufIndex = (bufIndex+1)%SMOOTH_SAMPLES;

    smoothedAccel={0,0,0}; smoothedGyro={0,0,0}; smoothedQuat={0,0,0,0};
    for(int i=0;i<SMOOTH_SAMPLES;i++){
      smoothedAccel.x+=accelBuffer[i].x; smoothedAccel.y+=accelBuffer[i].y; smoothedAccel.z+=accelBuffer[i].z;
      smoothedGyro.x+=gyroBuffer[i].x; smoothedGyro.y+=gyroBuffer[i].y; smoothedGyro.z+=gyroBuffer[i].z;
      smoothedQuat.w+=quatBuffer[i].w; smoothedQuat.x+=quatBuffer[i].x; smoothedQuat.y+=quatBuffer[i].y; smoothedQuat.z+=quatBuffer[i].z;
    }
    smoothedAccel.x/=SMOOTH_SAMPLES; smoothedAccel.y/=SMOOTH_SAMPLES; smoothedAccel.z/=SMOOTH_SAMPLES;
    smoothedGyro.x/=SMOOTH_SAMPLES; smoothedGyro.y/=SMOOTH_SAMPLES; smoothedGyro.z/=SMOOTH_SAMPLES;
    float mag = sqrt(smoothedQuat.w*smoothedQuat.w+smoothedQuat.x*smoothedQuat.x+smoothedQuat.y*smoothedQuat.y+smoothedQuat.z*smoothedQuat.z);
    smoothedQuat.w/=mag; smoothedQuat.x/=mag; smoothedQuat.y/=mag; smoothedQuat.z/=mag;
  }
}

// ===== PREDICTIVE VIRTUAL BONES =====
void updateVirtualBones(){
  for(int i=0;i<10;i++)
    virtualBoneQuat[i] = quatLerp(virtualBoneQuat[i], smoothedQuat, 0.3);
}

// ===== BROADCAST =====
void broadcast(){
  // Build FEC packet
  FECBlock b; int offset=0;
  memcpy(b.data+offset,&smoothedAccel,sizeof(Vec3)); offset+=sizeof(Vec3);
  memcpy(b.data+offset,&smoothedQuat,sizeof(Quat)); offset+=sizeof(Quat);
  for(int i=0;i<10;i++){ memcpy(b.data+offset,&virtualBoneQuat[i],sizeof(Quat)); offset+=sizeof(Quat); }
  memcpy(b.data+offset,&batteryVoltage,sizeof(float));
  encodeFEC(b);

  // UDP
  udp.beginPacket("255.255.255.255",udpPort);
  udp.write(b.data,128); udp.write(&b.parity,1);
  udp.endPacket();

  // OSC
  OSCMessage msg("/tracker");
  msg.add(WiFi.macAddress().c_str());
  msg.add(smoothedAccel.x); msg.add(smoothedAccel.y); msg.add(smoothedAccel.z);
  msg.add(smoothedQuat.w); msg.add(smoothedQuat.x); msg.add(smoothedQuat.y); msg.add(smoothedQuat.z);
  for(int i=0;i<10;i++){
    msg.add(virtualBoneQuat[i].w); msg.add(virtualBoneQuat[i].x);
    msg.add(virtualBoneQuat[i].y); msg.add(virtualBoneQuat[i].z);
  }
  msg.add(batteryVoltage);
  osc.send(msg,oscHostIP,oscHostPort);
  msg.empty();
}

// ===== MAIN LOOP =====
void loop(){
  static unsigned long lastMicros=0;
  readBattery();
  updateIMU();
  updateVirtualBones();

  // TDMA scheduling
  unsigned long now = micros();
  if((now%(SLOT_DURATION_US*TOTAL_TRACKERS)) >= mySlot*SLOT_DURATION_US &&
     (now-lastMicros)>=SLOT_DURATION_US){
    broadcast();
    lastMicros = now;
  }
}
