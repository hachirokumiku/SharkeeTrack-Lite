#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <MadgwickAHRS.h>
#include <EEPROM.h>
#include <OSCMessage.h>

// ===================================
// CONFIGURATION
// ===================================

#define IMU_INT1_PIN D5
#define UDP_PORT 9000
#define MAX_PEERS 10
#define BEACON_INTERVAL 100 // ms
#define FRAME_RATE 3200
#define IMU_UPDATE_RATE 3200

// Set this to the IP of your computer running the OSC server (e.g., SlimeVR)
IPAddress oscServerIP(192, 168, 1, 10);
const int oscServerPort = 9000;

// NTP sync interval to correct for clock drift
#define NTP_SYNC_INTERVAL 3600000 // 1 hour

// ===================================
// PERSONALIZED KINEMATIC MODEL
// ===================================

// Your specific body measurements
const float MY_HEIGHT_CM = 170.0;
const float MY_WAIST_INCHES = 34.0;

// Anatomical ratios for a statistically average human.
// Used to calculate bone lengths from your height.
const float BICEP_TO_HEIGHT_RATIO = 0.195;
const float FOREARM_TO_HEIGHT_RATIO = 0.170;
const float TORSO_TO_HEIGHT_RATIO = 0.33;
const float FEMUR_TO_HEIGHT_RATIO = 0.26;
const float TIBIA_TO_HEIGHT_RATIO = 0.25;

const float BICEP_LENGTH = MY_HEIGHT_CM * BICEP_TO_HEIGHT_RATIO / 100.0;
const float FOREARM_LENGTH = MY_HEIGHT_CM * FOREARM_TO_HEIGHT_RATIO / 100.0;
const float TORSO_LENGTH = MY_HEIGHT_CM * TORSO_TO_HEIGHT_RATIO / 100.0;
const float FEMUR_LENGTH = MY_HEIGHT_CM * FEMUR_TO_HEIGHT_RATIO / 100.0;
const float TIBIA_LENGTH = MY_HEIGHT_CM * TIBIA_TO_HEIGHT_RATIO / 100.0;

// Unique IDs for each body part. You will assign one of these to each tracker.
#define LIMB_TORSO 0
#define LIMB_BICEP_R 1
#define LIMB_FOREARM_R 2
#define LIMB_HAND_R 3
#define LIMB_FEMUR_R 4
#define LIMB_TIBIA_R 5

struct KinematicNode {
    uint8_t limbType;
    uint8_t upstreamID;
    float boneLength;
};

// ===================================
// NTP AND TIME SYNCHRONIZATION
// ===================================

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
unsigned long ntpSyncTime = 0;
long timeOffset = 0;

void syncNTP() {
    if (millis() - ntpSyncTime >= NTP_SYNC_INTERVAL) {
        timeClient.forceUpdate();
        long now_ms = millis();
        long ntp_ms = timeClient.getEpochTime() * 1000;
        timeOffset = ntp_ms - now_ms;
        ntpSyncTime = millis();
    }
}

unsigned long getSyncedTime() {
    return millis() + timeOffset;
}

// ===================================
// IMU AND EXTENDED KALMAN FILTER (EKF)
// ===================================

BMI270 imu;
Madgwick filter;
float deltaT = 1.0 / IMU_UPDATE_RATE;

struct Quat { float w, x, y, z; };
struct Vector3 { float x, y, z; };
struct ImuData { Quat quat; Vector3 accel; };

volatile bool imuReady = false;

class EKF {
private:
    float x_hat[7] = {1, 0, 0, 0, 0, 0, 0}; // State: [q_w, q_x, q_y, q_z, pos_x, pos_y, pos_z]
    float P[7][7] = {0}; // Error covariance
    const float Q = 0.01;
    const float R_accel = 0.5;
    const float R_quat = 0.1;
public:
    EKF() { for (int i = 0; i < 7; ++i) P[i][i] = 1.0; }
    void predict(float latency = 0.0) {
        float Q_adj = Q + latency * 0.1; // Increase process noise with latency
        for (int i = 0; i < 7; ++i) P[i][i] += Q_adj;
    }
    void update(const ImuData& measurement, float reliability) {
        float R_adj_accel = R_accel / reliability;
        float R_adj_quat = R_quat / reliability;
        x_hat[0] += (measurement.quat.w - x_hat[0]) * R_adj_quat;
        x_hat[1] += (measurement.quat.x - x_hat[1]) * R_adj_quat;
        x_hat[2] += (measurement.quat.y - x_hat[2]) * R_adj_quat;
        x_hat[3] += (measurement.quat.z - x_hat[3]) * R_adj_quat;
        x_hat[4] += (measurement.accel.x - x_hat[4]) * R_adj_accel;
        x_hat[5] += (measurement.accel.y - x_hat[5]) * R_adj_accel;
        x_hat[6] += (measurement.accel.z - x_hat[6]) * R_adj_accel;
    }
    Quat getQuat() const { return {x_hat[0], x_hat[1], x_hat[2], x_hat[3]}; }
    Vector3 getPos() const { return {x_hat[4], x_hat[5], x_hat[6]}; }
};

// ===================================
// P2P NETWORK AND DATA STRUCTURES
// ===================================

WiFiUDP udp;
uint8_t deviceID = 0;
unsigned long lastBeaconTime = 0;

struct Peer {
    uint8_t id;
    int rssi;
    float reliabilityFactor;
    Quat quat;
    Vector3 pos;
    unsigned long lastRxTime;
    EKF ekf;
};
Peer peers[MAX_PEERS];
uint8_t peerCount = 0;

// Custom P2P Data Packet for efficiency
struct P2PDataPacket {
    uint8_t id;
    unsigned long timestamp;
    uint32_t sequenceNum;
    ImuData imuData;
    Quat fusedQuat;
    Vector3 fusedPos;
    uint16_t crc;
};

// Fused data from local EKF and IK
Quat myFusedQuat;
Vector3 myFusedPos;
EKF myEKF;
KinematicNode myKinematicData = {LIMB_FOREARM_R, LIMB_BICEP_R, FOREARM_LENGTH};

// ===================================
// MATH AND CHECKSUM HELPERS
// ===================================

float quaternion_magnitude(const Quat& q) {
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

Quat quaternion_normalize(const Quat& q) {
    float mag = quaternion_magnitude(q);
    return {q.w / mag, q.x / mag, q.y / mag, q.z / mag};
}

// Function to rotate a 3D vector by a quaternion
Vector3 quaternion_rotate(const Quat& q, const Vector3& v) {
    Quat p = {0, v.x, v.y, v.z};
    Quat q_inv = {q.w, -q.x, -q.y, -q.z};
    Quat rotated = {0, 0, 0, 0};
    rotated.w = -q.x*p.x - q.y*p.y - q.z*p.z;
    rotated.x = q.w*p.x + q.y*p.z - q.z*p.y;
    rotated.y = q.w*p.y - q.x*p.z + q.z*p.x;
    rotated.z = q.w*p.z + q.x*p.y - q.y*p.x;
    Vector3 result;
    result.x = rotated.w*q_inv.x + rotated.x*q_inv.w + rotated.y*q_inv.z - rotated.z*q_inv.y;
    result.y = rotated.w*q_inv.y - rotated.x*q_inv.z + rotated.y*q_inv.w + rotated.z*q_inv.x;
    result.z = rotated.w*q_inv.z + rotated.x*q_inv.y - rotated.y*q_inv.x + rotated.z*q_inv.w;
    return result;
}

// CRC16 checksum for packet validation
uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i=0;i<len;i++){
        crc ^= (uint16_t)data[i]<<8;
        for (uint8_t j=0;j<8;j++)
            crc = (crc & 0x8000)? (crc<<1)^0x1021 : (crc<<1);
    }
    return crc;
}

// ===================================
// ARDUINO SETUP AND MAIN LOOP
// ===================================

void ICACHE_RAM_ATTR imuISR() {
    imuReady = true;
}

void setup() {
    Wire.begin();
    pinMode(IMU_INT1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imuISR, RISING);
    if (imu.begin() != BMI2_OK) while (1) delay(100);
    imu.enableFeature(BMI2_ACCEL | BMI2_GYRO, true);
    filter.begin(IMU_UPDATE_RATE);
    WiFi.mode(WIFI_STA);
    WiFi.begin("YOUR_SSID", "YOUR_PASSWORD");
    while (WiFi.status() != WL_CONNECTED) delay(10);
    udp.begin(UDP_PORT);
    timeClient.begin();
    EEPROM.begin(512);
    deviceID = EEPROM.read(0);
    if (deviceID == 0xFF) {
        deviceID = random(1, 255);
        EEPROM.write(0, deviceID);
        EEPROM.commit();
    }
    // Set my kinematic data based on my unique ID
    if (deviceID == LIMB_TORSO) myKinematicData = {LIMB_TORSO, 0, TORSO_LENGTH};
    if (deviceID == LIMB_BICEP_R) myKinematicData = {LIMB_BICEP_R, LIMB_TORSO, BICEP_LENGTH};
    if (deviceID == LIMB_FOREARM_R) myKinematicData = {LIMB_FOREARM_R, LIMB_BICEP_R, FOREARM_LENGTH};
}

void updateImuAndEKF() {
    if (!imuReady) return;
    imuReady = false;
    imu.readSensor();
    filter.updateIMU(imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az());
}

void sendDataPacket() {
    P2PDataPacket packet;
    packet.id = deviceID;
    packet.timestamp = getSyncedTime();
    packet.sequenceNum = millis();
    packet.imuData = { {filter.q0, filter.q1, filter.q2, filter.q3}, {imu.ax(), imu.ay(), imu.az()} };
    packet.fusedQuat = myFusedQuat;
    packet.fusedPos = myFusedPos;

    uint16_t packetCrc = crc16((const uint8_t*)&packet, sizeof(packet) - sizeof(uint16_t));
    packet.crc = packetCrc;

    for (int i = 0; i < peerCount; ++i) {
        udp.beginPacket(WiFi.localIP(), UDP_PORT);
        udp.write((const uint8_t*)&packet, sizeof(packet));
        udp.endPacket();
    }
}

void sendBeacon() {
    unsigned long now = millis();
    if (now - lastBeaconTime < BEACON_INTERVAL) return;
    lastBeaconTime = now;
    uint8_t beaconPayload[32];
    beaconPayload[0] = deviceID;
    udp.beginPacket(WiFi.localIP(), UDP_PORT);
    udp.write(beaconPayload, sizeof(beaconPayload));
    udp.endPacket();
}

float mapRssiToReliability(int rssi) {
    const int minRssi = -90;
    const int maxRssi = -40;
    if (rssi > maxRssi) return 1.0;
    if (rssi < minRssi) return 0.1;
    return 0.1 + (float)(rssi - minRssi) * (0.9f / (float)(maxRssi - minRsi));
}

void checkIncomingPackets() {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        uint8_t buffer[packetSize];
        udp.read(buffer, packetSize);
        P2PDataPacket* incomingPacket = (P2PDataPacket*)buffer;
        
        uint16_t receivedCrc = incomingPacket->crc;
        incomingPacket->crc = 0;
        if (crc16((const uint8_t*)incomingPacket, sizeof(P2PDataPacket) - sizeof(uint16_t)) != receivedCrc) {
            return;
        }

        uint8_t senderID = incomingPacket->id;
        int rssi = WiFi.RSSI();
        unsigned long packetTimestamp = incomingPacket->timestamp;
        
        float latency = (float)(getSyncedTime() - packetTimestamp) / 1000.0;
        if (latency < 0) latency = 0;

        int peerIdx = -1;
        for (int i = 0; i < peerCount; ++i) {
            if (peers[i].id == senderID) {
                peerIdx = i;
                break;
            }
        }
        if (peerIdx == -1 && peerCount < MAX_PEERS) {
            peerIdx = peerCount++;
            peers[peerIdx].id = senderID;
        }
        if (peerIdx != -1) {
            peers[peerIdx].rssi = rssi;
            peers[peerIdx].reliabilityFactor = mapRssiToReliability(rssi);
            peers[peerIdx].quat = incomingPacket->fusedQuat;
            peers[peerIdx].pos = incomingPacket->fusedPos;
            peers[peerIdx].lastRxTime = incomingPacket->timestamp;
            peers[peerIdx].ekf.predict(latency);
            peers[peerIdx].ekf.update(incomingPacket->imuData, peers[peerIdx].reliabilityFactor);
        }
    }
}

void solveDistributedIK() {
    myEKF.predict();
    myEKF.update({ {filter.q0, filter.q1, filter.q2, filter.q3}, {imu.ax(), imu.ay(), imu.az()} }, 1.0);
    myFusedQuat = myEKF.getQuat();
    myFusedPos = myEKF.getPos();

    int upstreamPeerIdx = -1;
    if (myKinematicData.upstreamID != 0) {
        for (int i = 0; i < peerCount; ++i) {
            if (peers[i].id == myKinematicData.upstreamID) {
                upstreamPeerIdx = i;
                break;
            }
        }
    }

    if (upstreamPeerIdx != -1) {
        Quat upstreamQuat = peers[upstreamPeerIdx].quat;
        Vector3 upstreamPos = peers[upstreamPeerIdx].pos;
        
        Vector3 boneVec = {myKinematicData.boneLength, 0, 0};
        
        Vector3 rotatedBone = quaternion_rotate(upstreamQuat, boneVec);
        
        myFusedPos.x = upstreamPos.x + rotatedBone.x;
        myFusedPos.y = upstreamPos.y + rotatedBone.y;
        myFusedPos.z = upstreamPos.z + rotatedBone.z;
    }
}

void sendOsc() {
    char address[32];
    sprintf(address, "/tracker/%d/rotation", deviceID);
    OSCMessage msg(address);
    msg.add(myFusedQuat.x).add(myFusedQuat.y).add(myFusedQuat.z).add(myFusedQuat.w);
    udp.beginPacket(oscServerIP, oscServerPort);
    msg.send(udp);
    udp.endPacket();
    msg.empty();
    sprintf(address, "/tracker/%d/position", deviceID);
    OSCMessage posMsg(address);
    posMsg.add(myFusedPos.x).add(myFusedPos.y).add(myFusedPos.z);
    udp.beginPacket(oscServerIP, oscServerPort);
    posMsg.send(udp);
    udp.endPacket();
}

void loop() {
    syncNTP();
    updateImuAndEKF();
    checkIncomingPackets();
    solveDistributedIK();
    sendDataPacket();
    sendBeacon();
    sendOsc();
}
