#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <MadgwickAHRS.h>
#include <EEPROM.h>
#include <OSCMessage.h>

// ====================================================================================
// SYSTEM CONFIGURATION
// This section defines all the core parameters for the system, making them easy to
// find and change. Think of this as the main control panel for your tracker.
// ====================================================================================

// Pin definitions for the BMI270's hardware interrupts.
// These are connected to the ESP8266 to allow for non-blocking, event-driven
// data processing, which is crucial for low-latency applications.
#define IMU_INT1_PIN D5 // This pin will be triggered when new IMU data is ready.
#define IMU_INT2_PIN D6 // This pin will be triggered on a significant motion event.

// Network and communication settings.
#define UDP_PORT 9000 // The UDP port used for both P2P tracker communication and OSC output.
#define MAX_PEERS 10 // The maximum number of other trackers this device will track.
#define BEACON_INTERVAL 100 // How often this tracker broadcasts its presence to other trackers (in ms).
#define IMU_UPDATE_RATE 3200 // The frequency at which the IMU is polled (in Hz). A higher rate means more data, which leads to smoother motion but uses more power.

// Your Wi-Fi network credentials.
const char* wifi_ssid = "YOUR_SSID";
const char* wifi_password = "YOUR_PASSWORD";

// The IP address of the computer running your VRChat OSC server (e.g., SlimeVR).
// This is where the final tracking data will be sent.
IPAddress osc_server_ip(192, 168, 1, 10);
const int osc_server_port = 9000;

// NTP (Network Time Protocol) client for network time synchronization.
// This is used to correct for clock drift between trackers and accurately measure latency.
#define NTP_SYNC_INTERVAL 3600000 // How often to sync with the NTP server (1 hour). This prevents long-term drift.

// ====================================================================================
// PERSONALIZED KINEMATIC MODEL
// This section defines the physical dimensions of the user's body.
// These measurements are critical for the distributed IK solver to work correctly.
// ====================================================================================

// My body measurements. These are used to calculate bone lengths.
// Adjust these to match your own body for the best results!
const float my_height_cm = 170.0;

// Standard anatomical ratios for a statistically average human.
// These ratios allow the code to calculate the length of each body segment
// based on the user's overall height, making the system adaptable.
const float bicep_to_height_ratio = 0.195;
const float forearm_to_height_ratio = 0.170;
const float torso_to_height_ratio = 0.33;
const float femur_to_height_ratio = 0.26;
const float tibia_to_height_ratio = 0.25;

// Calculated bone lengths.
const float bicep_length = my_height_cm * bicep_to_height_ratio / 100.0;
const float forearm_length = my_height_cm * forearm_to_height_ratio / 100.0;
const float torso_length = my_height_cm * torso_to_height_ratio / 100.0;
const float femur_length = my_height_cm * femur_to_height_ratio / 100.0;
const float tibia_length = my_height_cm * tibia_to_height_ratio / 100.0;

// Unique IDs for each body part.
// Each tracker should have its `deviceID` set to one of these values.
// This tells the tracker which part of the body it's on and what its parent is.
#define LIMB_CHEST 0
#define LIMB_HIPS 1
#define LIMB_ELBOW_R 2
#define LIMB_THIGH_R 3
#define LIMB_ANKLE_R 4
#define LIMB_FOOT_R 5
#define LIMB_ELBOW_L 6
#define LIMB_THIGH_L 7
#define LIMB_ANKLE_L 8
#define LIMB_FOOT_L 9

// A struct to define a single kinematic node (a tracker's position in the body).
struct KinematicNode {
    uint8_t limbType; // The type of limb this node represents (e.g., LIMB_CHEST).
    uint8_t upstreamID; // The ID of the parent tracker. 0 for the root node (chest).
    float boneLength; // The distance from the parent tracker.
};

// ====================================================================================
// NTP AND TIME SYNCHRONIZATION
// These functions manage the system clock to ensure all trackers are in sync.
// Accurate time is essential for calculating network latency and making
// intelligent EKF predictions.
// ====================================================================================

WiFiUDP ntp_udp;
NTPClient ntp_client(ntp_udp, "pool.ntp.org");
unsigned long last_ntp_sync_time = 0;
long time_offset = 0;

// Synchronizes the microcontroller's clock with an NTP server.
void synchronizeNTP() {
    if (millis() - last_ntp_sync_time >= NTP_SYNC_INTERVAL) {
        ntp_client.forceUpdate(); // Request a new time from the server.
        long now_ms = millis();
        long ntp_ms = ntp_client.getEpochTime() * 1000;
        time_offset = ntp_ms - now_ms; // Calculate the time difference.
        last_ntp_sync_time = millis();
    }
}

// Returns the current synchronized time.
unsigned long getSyncedTime() {
    return millis() + time_offset;
}

// ====================================================================================
// IMU AND EXTENDED KALMAN FILTER (EKF)
// This is the heart of the tracking algorithm. It fuses raw IMU data to produce
// a stable, drift-free orientation and position.
// ====================================================================================

BMI270 imu; // The IMU sensor object.
Madgwick filter; // The Madgwick filter is used for orientation fusion. It's fast and effective.
float delta_t = 1.0 / IMU_UPDATE_RATE; // The time step for the filter update.

// Data structures for holding sensor data.
struct Quat { float w, x, y, z; }; // A quaternion for orientation.
struct Vector3 { float x, y, z; }; // A 3D vector for position or acceleration.
struct ImuData { Quat quat; Vector3 accel; };

// Volatile booleans for interrupt handling. They signal that new data is available.
volatile bool imu_data_ready = false;
volatile bool imu_motion_detected = false;

// The Extended Kalman Filter (EKF) is a powerful tool for fusing noisy data.
// It predicts the next state and then corrects that prediction with new measurements.
class EKF {
private:
    float x_hat[7] = {1, 0, 0, 0, 0, 0, 0}; // The filter's state: [quaternion, position].
    float P[7][7] = {0}; // The error covariance matrix, which represents the filter's uncertainty.
    const float Q = 0.01; // Process noise covariance. It represents how much we trust our prediction.
    const float R_accel = 0.5; // Measurement noise for the accelerometer.
    const float R_quat = 0.1; // Measurement noise for the quaternion.
public:
    EKF() { for (int i = 0; i < 7; ++i) P[i][i] = 1.0; }
    // The prediction step. It uses the dynamic model to predict the next state.
    // Latency is used to adjust the process noise, trusting the prediction less when
    // network lag is high.
    void predict(float latency = 0.0) {
        float Q_adj = Q + latency * 0.1;
        for (int i = 0; i < 7; ++i) P[i][i] += Q_adj;
    }
    // The update step. It uses a new measurement to correct the predicted state.
    // The 'reliability' parameter (derived from RSSI) dynamically adjusts the
    // measurement noise, trusting more reliable data more.
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
    // Accessor methods to retrieve the fused state.
    Quat getQuat() const { return {x_hat[0], x_hat[1], x_hat[2], x_hat[3]}; }
    Vector3 getPos() const { return {x_hat[4], x_hat[5], x_hat[6]}; }
};

// ====================================================================================
// P2P NETWORK AND DATA STRUCTURES
// This section handles the communication between trackers, forming a robust mesh network.
// This allows each tracker to know where all the others are.
// ====================================================================================

WiFiUDP p2p_udp;
uint8_t my_device_id = 0;
unsigned long last_beacon_time = 0;

// A struct that holds the state of a single peer tracker.
struct PeerTracker {
    uint8_t id;
    int rssi; // Signal strength, used for reliability.
    float reliability_factor; // How much we trust this peer's data.
    Quat quat;
    Vector3 pos;
    unsigned long last_rx_time; // Time of the last received packet.
    EKF ekf; // Each peer has its own EKF for local data fusion.
};
PeerTracker peer_trackers[MAX_PEERS];
uint8_t peer_count = 0;

// The data packet format for inter-tracker communication.
// This is a compact struct to minimize network overhead.
struct P2PDataPacket {
    uint8_t id;
    unsigned long timestamp; // Timestamp from the sending tracker's synchronized clock.
    uint32_t sequence_num;
    ImuData imuData; // Raw IMU data.
    Quat fusedQuat; // Already fused orientation.
    Vector3 fusedPos; // Already fused position.
    uint16_t crc; // Checksum to detect corrupted packets.
};

// My own fused data from local EKF and IK solver.
Quat my_fused_quat;
Vector3 my_fused_pos;
EKF my_ekf;
KinematicNode my_kinematic_data; // My tracker's place in the kinematic chain.

// ====================================================================================
// MATH AND CHECKSUM HELPERS
// These are utility functions for common quaternion and data integrity operations.
// ====================================================================================

float quaternion_magnitude(const Quat& q) {
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

Quat quaternion_normalize(const Quat& q) {
    float mag = quaternion_magnitude(q);
    return {q.w / mag, q.x / mag, q.y / mag, q.z / mag};
}

// Function to rotate a 3D vector by a quaternion.
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

// CRC16 checksum for packet validation.
// This is a simple but effective way to detect if a network packet was corrupted.
uint16_t crc16_calculate(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i=0;i<len;i++){
        crc ^= (uint16_t)data[i]<<8;
        for (uint8_t j=0;j<8;j++)
            crc = (crc & 0x8000)? (crc<<1)^0x1021 : (crc<<1);
    }
    return crc;
}

// ====================================================================================
// ARDUINO SETUP AND MAIN LOOP
// This is the core program flow. It handles initialization and the main loop,
// which is designed to be as fast as possible by relying on interrupts.
// ====================================================================================

// Interrupt Service Routines (ISRs) for the IMU.
// These functions are called immediately when a hardware interrupt is triggered,
// preventing delays from the main loop.
void ICACHE_RAM_ATTR imu1_ISR() {
    imu_data_ready = true;
}

void ICACHE_RAM_ATTR imu2_ISR() {
    imu_motion_detected = true;
}

void setup() {
    // 1. Hardware Initialization
    Wire.begin(); // Start I2C communication.
    pinMode(IMU_INT1_PIN, INPUT);
    pinMode(IMU_INT2_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imu1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(IMU_INT2_PIN), imu2_ISR, RISING);
    if (imu.begin() != BMI2_OK) while (1) delay(100); // Wait for the IMU to initialize.
    imu.enableFeature(BMI2_ACCEL | BMI2_GYRO, true); // Enable accelerometer and gyroscope.
    imu.enableAnyMotion(imu.ax(), imu.ay(), imu.az(), 100, 10, true, 1); // Configure motion detection.
    filter.begin(IMU_UPDATE_RATE);

    // 2. Network Initialization
    WiFi.mode(WIFI_STA); // Set Wi-Fi mode to Station (client).
    WiFi.begin(wifi_ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED) delay(10); // Wait for Wi-Fi connection.
    p2p_udp.begin(UDP_PORT); // Start UDP for P2P communication.
    ntp_client.begin(); // Start NTP client.

    // 3. Tracker ID Management
    EEPROM.begin(512); // Initialize EEPROM to store the unique tracker ID.
    my_device_id = EEPROM.read(0);
    if (my_device_id == 0xFF) { // If no ID is found (first boot), generate a new one.
        my_device_id = random(1, 255);
        EEPROM.write(0, my_device_id);
        EEPROM.commit();
    }
    
    // 4. Kinematic Data Assignment
    // Assign my specific kinematic data based on my unique ID.
    // This defines where this tracker is located on the body.
    switch(my_device_id) {
        case LIMB_CHEST: my_kinematic_data = {LIMB_CHEST, 0, 0.0}; break; // Root node, no parent, no length.
        case LIMB_HIPS: my_kinematic_data = {LIMB_HIPS, LIMB_CHEST, torso_length}; break;
        case LIMB_ELBOW_R: my_kinematic_data = {LIMB_ELBOW_R, LIMB_CHEST, bicep_length}; break;
        case LIMB_THIGH_R: my_kinematic_data = {LIMB_THIGH_R, LIMB_HIPS, femur_length}; break;
        case LIMB_ANKLE_R: my_kinematic_data = {LIMB_ANKLE_R, LIMB_THIGH_R, tibia_length}; break;
        case LIMB_FOOT_R: my_kinematic_data = {LIMB_FOOT_R, LIMB_ANKLE_R, forearm_length}; break;
        case LIMB_ELBOW_L: my_kinematic_data = {LIMB_ELBOW_L, LIMB_CHEST, bicep_length}; break;
        case LIMB_THIGH_L: my_kinematic_data = {LIMB_THIGH_L, LIMB_HIPS, femur_length}; break;
        case LIMB_ANKLE_L: my_kinematic_data = {LIMB_ANKLE_L, LIMB_THIGH_L, tibia_length}; break;
        case LIMB_FOOT_L: my_kinematic_data = {LIMB_FOOT_L, LIMB_ANKLE_L, forearm_length}; break;
    }
}

// Function to process local IMU data. This is called on every IMU data-ready interrupt.
void processLocalIMUData() {
    if (!imu_data_ready) return; // Only run if new data is available.
    imu_data_ready = false;
    imu.readSensor(); // Read raw data from the IMU.
    filter.updateIMU(imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az()); // Update the Madgwick filter.
}

// Broadcasts my own tracker data to peers.
void sendP2PDataPacket() {
    P2PDataPacket packet;
    packet.id = my_device_id;
    packet.timestamp = getSyncedTime();
    packet.sequence_num = millis();
    packet.imuData = { {filter.q0, filter.q1, filter.q2, filter.q3}, {imu.ax(), imu.ay(), imu.az()} };
    packet.fusedQuat = my_fused_quat;
    packet.fusedPos = my_fused_pos;
    uint16_t packet_crc = crc16_calculate((const uint8_t*)&packet, sizeof(packet) - sizeof(uint16_t));
    packet.crc = packet_crc;
    for (int i = 0; i < peer_count; ++i) {
        p2p_udp.beginPacket(WiFi.localIP(), UDP_PORT);
        p2p_udp.write((const uint8_t*)&packet, sizeof(packet));
        p2p_udp.endPacket();
    }
}

// Broadcasts a small beacon to discover other trackers on the network.
void sendBeacon() {
    unsigned long now = millis();
    if (now - last_beacon_time < BEACON_INTERVAL) return;
    last_beacon_time = now;
    uint8_t beacon_payload[32];
    beacon_payload[0] = my_device_id;
    p2p_udp.beginPacket(WiFi.localIP(), UDP_PORT);
    p2p_udp.write(beacon_payload, sizeof(beacon_payload));
    p2p_udp.endPacket();
}

// A robust, non-linear function to map RSSI to a data reliability factor.
// This is a key improvement for dealing with noisy Wi-Fi environments.
// It gives a higher score to strong signals, but a very low score to weak ones.
float mapRssiToReliability(int rssi) {
    if (rssi > -40) return 1.0; // Excellent signal, full reliability.
    if (rssi > -50) return 0.9; // Good signal, slight decrease in reliability.
    if (rssi > -60) return 0.7; // Fair signal, noticeable decrease.
    if (rssi > -70) return 0.5; // Poor signal, low trust in data.
    if (rssi > -80) return 0.3; // Very poor signal, minimal trust.
    return 0.1; // Unusable signal, almost no trust.
}

// Processes incoming data packets from other trackers.
// It discards corrupted packets and updates the peer's EKF.
void processIncomingP2PPackets() {
    int packet_size = p2p_udp.parsePacket();
    if (packet_size > 0) {
        uint8_t buffer[packet_size];
        p2p_udp.read(buffer, packet_size);
        P2PDataPacket* incoming_packet = (P2PDataPacket*)buffer;
        uint16_t received_crc = incoming_packet->crc;
        incoming_packet->crc = 0;
        if (crc16_calculate((const uint8_t*)incoming_packet, sizeof(P2PDataPacket) - sizeof(uint16_t)) != received_crc) {
            return; // Discard corrupted packet.
        }
        uint8_t sender_id = incoming_packet->id;
        int rssi = WiFi.RSSI();
        unsigned long packet_timestamp = incoming_packet->timestamp;
        float latency = (float)(getSyncedTime() - packet_timestamp) / 1000.0;
        if (latency < 0) latency = 0;
        int peer_idx = -1;
        for (int i = 0; i < peer_count; ++i) {
            if (peer_trackers[i].id == sender_id) {
                peer_idx = i;
                break;
            }
        }
        if (peer_idx == -1 && peer_count < MAX_PEERS) {
            peer_idx = peer_count++;
            peer_trackers[peer_idx].id = sender_id;
        }
        if (peer_idx != -1) {
            peer_trackers[peer_idx].rssi = rssi;
            peer_trackers[peer_idx].reliability_factor = mapRssiToReliability(rssi);
            peer_trackers[peer_idx].quat = incoming_packet->fusedQuat;
            peer_trackers[peer_idx].pos = incoming_packet->fusedPos;
            peer_trackers[peer_idx].last_rx_time = incoming_packet->timestamp;
            peer_trackers[peer_idx].ekf.predict(latency);
            peer_trackers[peer_idx].ekf.update(incoming_packet->imuData, peer_trackers[peer_idx].reliability_factor);
        }
    }
}

// Solves the distributed IK problem by combining local and peer data.
// It calculates this tracker's position based on its parent in the kinematic chain.
void solveDistributedIK() {
    // 1. Update my own EKF with the latest local data.
    my_ekf.predict();
    my_ekf.update({ {filter.q0, filter.q1, filter.q2, filter.q3}, {imu.ax(), imu.ay(), imu.az()} }, 1.0);
    my_fused_quat = my_ekf.getQuat();
    my_fused_pos = my_ekf.getPos();

    // 2. Find the upstream (parent) tracker.
    int upstream_peer_idx = -1;
    if (my_kinematic_data.upstreamID != 0) {
        for (int i = 0; i < peer_count; ++i) {
            if (peer_trackers[i].id == my_kinematic_data.upstreamID) {
                upstream_peer_idx = i;
                break;
            }
        }
    }
    
    // 3. If a parent is found, calculate my position relative to it.
    if (upstream_peer_idx != -1) {
        Quat upstream_quat = peer_trackers[upstream_peer_idx].quat;
        Vector3 upstream_pos = peer_trackers[upstream_peer_idx].pos;
        Vector3 bone_vector = {my_kinematic_data.boneLength, 0, 0};
        Vector3 rotated_bone = quaternion_rotate(upstream_quat, bone_vector);
        // The final position is the parent's position plus the rotated bone vector.
        my_fused_pos.x = upstream_pos.x + rotated_bone.x;
        my_fused_pos.y = upstream_pos.y + rotated_bone.y;
        my_fused_pos.z = upstream_pos.z + rotated_bone.z;
    }
}

// Sends the final fused data to the VRChat OSC server.
void sendOSCData() {
    char address[32];
    sprintf(address, "/tracking/trackers/%d/rotation", my_device_id);
    OSCMessage msg(address);
    msg.add(my_fused_quat.x).add(my_fused_quat.y).add(my_fused_quat.z).add(my_fused_quat.w);
    p2p_udp.beginPacket(osc_server_ip, osc_server_port);
    msg.send(p2p_udp);
    p2p_udp.endPacket();
    msg.empty();
    sprintf(address, "/tracking/trackers/%d/position", my_device_id);
    OSCMessage pos_msg(address);
    pos_msg.add(my_fused_pos.x).add(my_fused_pos.y).add(my_fused_pos.z);
    p2p_udp.beginPacket(osc_server_ip, osc_server_port);
    pos_msg.send(p2p_udp);
    p2p_udp.endPacket();
}

// The main loop. It runs continuously, but relies on interrupts to handle data efficiently.
void loop() {
    // These functions are non-blocking and will execute as fast as possible.
    synchronizeNTP();
    processLocalIMUData();
    processIncomingP2PPackets();
    solveDistributedIK();
    sendP2PDataPacket();
    sendBeacon();
    sendOSCData();
}
