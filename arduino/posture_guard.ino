#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define CALIBRATION_SAMPLES 1000
#define PERFORMANCE_REPORT_INTERVAL 10000

#define BT_RX_PIN 2
#define BT_TX_PIN 3
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);

#define EEPROM_GYRO_X_OFFSET 0
#define EEPROM_GYRO_Y_OFFSET 2
#define EEPROM_GYRO_Z_OFFSET 4
#define EEPROM_ACCEL_Z_OFFSET 6
#define EEPROM_CALIBRATION_FLAG 8

struct PostureThresholds {
    int roll_good = 5.0;
    int roll_mid = 15.0;
    int pitch_good_min = -15.0;
    int pitch_good_max = -5.0;
    int pitch_mid_min = -25.0;
    int pitch_mid_max = 5.0;
    int yaw_good = 10.0;
    int yaw_mid = 25.0;
};

struct AlertThresholds {
    unsigned long bad_posture_duration = 30000;
    unsigned long reminder_interval = 300000;
    int excessive_tilt_threshold = 30.0;
};

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

int16_t gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0, accelZOffset = 0;
bool isCalibrated = false;

static int  lastTime = 0;
static unsigned long sampleCount = 0;
static unsigned long lastSampleRateReport = 0;
static float avgSampleRate = 0.0;

static int fifoOverflowCount = 0;
static int dmpErrorCount = 0;

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

PostureThresholds customThresholds;
AlertThresholds alertSettings;

float prev_roll = 0, prev_pitch = 0, prev_yaw = 0;

bool debugMode = false;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    bluetooth.begin(9600);
    delay(1000);
    bluetooth.print("INIT:");
    bluetooth.print(",");
    
    while (!Serial);
    mpu.initialize();
    
    if (!loadCalibrationFromEEPROM()) {
        Serial.println("No valid calibration found. Starting auto-calibration...");
        Serial.println("Please keep the device still for calibration!");
        delay(3000);
        calibrateMPU();
    } else {
        isCalibrated = true;
        Serial.println("Using saved calibration data");
    }
    
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        bluetooth.println("STATUS:READY");
    }
}

void calibrateMPU() {
    Serial.println("Starting MPU6050 calibration...");
    bluetooth.println("STATUS:CALIBRATING");
    
    long gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
    int samples = CALIBRATION_SAMPLES;
    
    for (int i = 0; i < samples; i++) {
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
        
        gx += gx_raw;
        gy += gy_raw;
        gz += gz_raw;
        ax += ax_raw;
        ay += ay_raw;
        az += az_raw;
        
        if (i % 100 == 0) {
            Serial.print("Calibration progress: ");
            Serial.print((i * 100) / samples);
            Serial.println("%");
        }
        delay(2);
    }
    
    gyroXOffset = gx / samples;
    gyroYOffset = gy / samples;
    gyroZOffset = gz / samples;
    accelZOffset = (az / samples) - 16384;
    
    EEPROM.put(EEPROM_GYRO_X_OFFSET, gyroXOffset);
    EEPROM.put(EEPROM_GYRO_Y_OFFSET, gyroYOffset);
    EEPROM.put(EEPROM_GYRO_Z_OFFSET, gyroZOffset);
    EEPROM.put(EEPROM_ACCEL_Z_OFFSET, accelZOffset);
    EEPROM.put(EEPROM_CALIBRATION_FLAG, (byte)0xAB);
    
    mpu.setXGyroOffset(gyroXOffset);
    mpu.setYGyroOffset(gyroYOffset);
    mpu.setZGyroOffset(gyroZOffset);
    mpu.setZAccelOffset(16384 + accelZOffset);
    
    isCalibrated = true;
    
    bluetooth.print("CALIBRATION:COMPLETE,");
    bluetooth.print(gyroXOffset); bluetooth.print(",");
    bluetooth.print(gyroYOffset); bluetooth.print(",");
    bluetooth.print(gyroZOffset); bluetooth.print(",");
    bluetooth.println(accelZOffset);
}

void advancedCalibration() {
    bluetooth.println("STATUS:ADVANCED_CALIBRATION_START");
    
    bluetooth.println("CALIBRATION:PLACE_FLAT");
    while (!Serial.available() && !bluetooth.available()) delay(100);
    if (Serial.available()) Serial.read();
    if (bluetooth.available()) bluetooth.readString();
    
    long gx1=0, gy1=0, gz1=0, ax1=0, ay1=0, az1=0;
    for (int i=0; i<500; i++) {
        int16_t ax,ay,az,gx,gy,gz;
        mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
        gx1+=gx; gy1+=gy; gz1+=gz; ax1+=ax; ay1+=ay; az1+=az;
        delay(2);
    }
    
    bluetooth.println("CALIBRATION:PLACE_UPRIGHT");
    while (!Serial.available() && !bluetooth.available()) delay(100);
    if (Serial.available()) Serial.read();
    if (bluetooth.available()) bluetooth.readString();
    
    long gx2=0, gy2=0, gz2=0, ax2=0, ay2=0, az2=0;
    for (int i=0; i<500; i++) {
        int16_t ax,ay,az,gx,gy,gz;
        mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
        gx2+=gx; gy2+=gy; gz2+=gz; ax2+=ax; ay2+=ay; az2+=az;
        delay(2);
    }
    
    int16_t newGyroXOffset = (gx1 + gx2) / 1000;
    int16_t newGyroYOffset = (gy1 + gy2) / 1000;
    int16_t newGyroZOffset = (gz1 + gz2) / 1000;
    
    EEPROM.put(0, newGyroXOffset);
    EEPROM.put(2, newGyroYOffset);
    EEPROM.put(4, newGyroZOffset);
    EEPROM.put(8, (byte)0xCD);
    
    gyroXOffset = newGyroXOffset;
    gyroYOffset = newGyroYOffset;
    gyroZOffset = newGyroZOffset;
    
    mpu.setXGyroOffset(gyroXOffset);
    mpu.setYGyroOffset(gyroYOffset);
    mpu.setZGyroOffset(gyroZOffset);
    
    bluetooth.println("STATUS:ADVANCED_CALIBRATION_COMPLETE");
}

bool loadCalibrationFromEEPROM() {
    byte calibFlag;
    EEPROM.get(EEPROM_CALIBRATION_FLAG, calibFlag);
    
    if (calibFlag == 0xAB || calibFlag == 0xCD) {
        EEPROM.get(EEPROM_GYRO_X_OFFSET, gyroXOffset);
        EEPROM.get(EEPROM_GYRO_Y_OFFSET, gyroYOffset);
        EEPROM.get(EEPROM_GYRO_Z_OFFSET, gyroZOffset);
        EEPROM.get(EEPROM_ACCEL_Z_OFFSET, accelZOffset);
        
        mpu.setXGyroOffset(gyroXOffset);
        mpu.setYGyroOffset(gyroYOffset);
        mpu.setZGyroOffset(gyroZOffset);
        mpu.setZAccelOffset(16384 + accelZOffset);
        
        bluetooth.println("STATUS:CALIBRATION_LOADED");
        return true;
    }
    return false;
}

void updateSampleRate() {
    sampleCount++;
    unsigned long now = millis();
    
    if (lastTime > 0) {
        float dt = (now - lastTime) / 1000.0;
        if (dt > 0) {
            float currentRate = 1.0 / dt;
            avgSampleRate = (avgSampleRate * 0.95) + (currentRate * 0.05);
        }
    }
    lastTime = now;
    
    if (now - lastSampleRateReport > 5000) {
        if (debugMode) {
        }
        
        bluetooth.print("SAMPLE_RATE:");
        bluetooth.println(avgSampleRate, 1);
        
        lastSampleRateReport = now;
    }
}

String evaluatePostureAdvanced(float roll, float pitch, float yaw, PostureThresholds& thresholds) {
    String rollStatus = "";
    String pitchStatus = "";
    String yawStatus = "";
    
    if (abs(roll) <= thresholds.roll_good) {
        rollStatus = "GOOD";
    } else if (abs(roll) <= thresholds.roll_mid) {
        rollStatus = "MID";
    } else {
        rollStatus = "BAD";
    }
    
    if (pitch >= thresholds.pitch_good_min && pitch <= thresholds.pitch_good_max) {
        pitchStatus = "GOOD";
    } else if (pitch >= thresholds.pitch_mid_min && pitch <= thresholds.pitch_mid_max) {
        pitchStatus = "MID";
    } else {
        pitchStatus = "BAD";
        if (pitch > thresholds.pitch_mid_max) {
            pitchStatus += "_FORWARD_LEAN";
        }
    }
    
    if (abs(yaw) <= thresholds.yaw_good) {
        yawStatus = "GOOD";
    } else if (abs(yaw) <= thresholds.yaw_mid) {
        yawStatus = "MID";
    } else {
        yawStatus = "BAD";
    }
    
    int goodCount = 0, midCount = 0, badCount = 0;
    
    if (rollStatus == "GOOD") goodCount++;
    else if (rollStatus == "MID") midCount++;
    else badCount++;
    
    if (pitchStatus.startsWith("GOOD")) goodCount++;
    else if (pitchStatus.startsWith("MID")) midCount++;
    else badCount++;
    
    if (yawStatus == "GOOD") goodCount++;
    else if (yawStatus == "MID") midCount++;
    else badCount++;
    
    String overallPosture = "";
    if (badCount > 0) {
        overallPosture = "BAD";
        if (pitchStatus.indexOf("FORWARD_LEAN") >= 0) {
            overallPosture += "_FORWARD_HEAD";
        }
    } else if (goodCount >= 2) {
        overallPosture = "GOOD";
    } else {
        overallPosture = "MID";
    }
    
    return overallPosture;
}

int calculatePostureScore(float roll, float pitch, float yaw) {
    int score = 100;
    
    score -= (int)(abs(roll) * 2);
    
    float optimal_pitch = -10.0;
    score -= (int)(abs(pitch - optimal_pitch) * 1.5);
    
    score -= (int)(abs(yaw) * 1.5);
    
    if (score < 0) score = 0;
    if (score > 100) score = 100;
    
    return score;
}

String getDeviceOrientation(float roll, float pitch, float yaw) {
    String orientation = "";
    
    if (roll > 10) orientation += "TILT_RIGHT ";
    else if (roll < -10) orientation += "TILT_LEFT ";
    else orientation += "LEVEL ";
    
    if (pitch > 15) orientation += "LEAN_FORWARD ";
    else if (pitch < -15) orientation += "LEAN_BACK ";
    else orientation += "UPRIGHT ";
    
    if (yaw > 20) orientation += "TURN_RIGHT";
    else if (yaw < -20) orientation += "TURN_LEFT";
    else orientation += "FACING_FORWARD";
    
    return orientation;
}

void sendBluetoothData(float roll, float pitch, float yaw, String postureResult, String deviceOrientation) {
    unsigned long timestamp_ms = millis();
    unsigned long timestamp_us = micros();
    
    bluetooth.print("{");
    bluetooth.print("\"roll\":"); bluetooth.print(roll, 2); bluetooth.print(",");
    bluetooth.print("\"pitch\":"); bluetooth.print(pitch, 2); bluetooth.print(",");
    bluetooth.print("\"yaw\":"); bluetooth.print(yaw, 2); bluetooth.print(",");
    bluetooth.print("\"posture\":\""); bluetooth.print(postureResult); bluetooth.print("\",");
    bluetooth.print("\"orientation\":\""); bluetooth.print(deviceOrientation); bluetooth.print("\",");
    bluetooth.print("\"timestamp_ms\":"); bluetooth.print(timestamp_ms); bluetooth.print(",");
    bluetooth.print("\"timestamp_us\":"); bluetooth.print(timestamp_us); bluetooth.print(",");
    bluetooth.print("\"sample_rate\":"); bluetooth.print(avgSampleRate, 1); bluetooth.print(",");
    bluetooth.print("\"calibrated\":"); bluetooth.print(isCalibrated ? "true" : "false"); bluetooth.print(",");
    bluetooth.print("\"fifo_errors\":"); bluetooth.print(fifoOverflowCount); bluetooth.print(",");
    bluetooth.print("\"sample_count\":"); bluetooth.print(sampleCount);
    
    int score = calculatePostureScore(roll, pitch, yaw);
    bluetooth.print(",\"score\":");
    bluetooth.print(score);
    
    bluetooth.println("}");
}

void processBluetoothCommands() {
    if (bluetooth.available()) {
        String command = bluetooth.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command == "STATUS") {
            bluetooth.print("STATUS:ACTIVE,");
            bluetooth.print(avgSampleRate, 1);
            bluetooth.print("Hz,");
            bluetooth.print(isCalibrated ? "CALIBRATED" : "NOT_CALIBRATED");
            
        } else if (command == "CALIBRATE") {
            bluetooth.println("STATUS:CALIBRATION_STARTED");
            calibrateMPU();
            
        } else if (command == "ADVANCED_CAL") {
            bluetooth.println("STATUS:ADVANCED_CALIBRATION_STARTED");
            advancedCalibration();
            
        } else if (command == "GET_CONFIG") {
            sendConfigUpdate();
            
        } else {
            bluetooth.print("ERROR:UNKNOWN_COMMAND:");
            bluetooth.println(command);
        }
    }
}

void sendConfigUpdate() {
    bluetooth.print("CONFIG:{");
    bluetooth.print("\"roll_good\":"); bluetooth.print(customThresholds.roll_good); bluetooth.print(",");
    bluetooth.print("\"roll_mid\":"); bluetooth.print(customThresholds.roll_mid); bluetooth.print(",");
    bluetooth.print("\"pitch_good_min\":"); bluetooth.print(customThresholds.pitch_good_min); bluetooth.print(",");
    bluetooth.print("\"pitch_good_max\":"); bluetooth.print(customThresholds.pitch_good_max); bluetooth.print(",");
    bluetooth.print("\"pitch_mid_min\":"); bluetooth.print(customThresholds.pitch_mid_min); bluetooth.print(",");
    bluetooth.print("\"pitch_mid_max\":"); bluetooth.print(customThresholds.pitch_mid_max); bluetooth.print(",");
    bluetooth.print("\"yaw_good\":"); bluetooth.print(customThresholds.yaw_good); bluetooth.print(",");
    bluetooth.print("\"yaw_mid\":"); bluetooth.print(customThresholds.yaw_mid);
    bluetooth.println("}");
}

void loop() {
    if (!dmpReady) return;

    PostureThresholds thresholds;

    processBluetoothCommands();

    while (!mpuInterrupt && fifoCount < packetSize) {
        if (Serial.available() || bluetooth.available()) {
            break;
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        if (debugMode) {
            Serial.println("FIFO overflow detected and cleared");
        }
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        updateSampleRate();

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        float roll = ypr[2] * 180/M_PI;
        float pitch = ypr[1] * 180/M_PI;
        float yaw = ypr[0] * 180/M_PI;
        
        if (sampleCount > 10) {
        }
        
        prev_roll = roll;
        prev_pitch = pitch;
        prev_yaw = yaw;

        String postureResult = evaluatePostureAdvanced(roll, pitch, yaw, thresholds);
        String deviceOrientation = getDeviceOrientation(roll, pitch, yaw);
        
        sendBluetoothData(roll, pitch, yaw, postureResult, deviceOrientation);
        
        static unsigned long lastScientificSend = 0;
        if (millis() - lastScientificSend > 5000) {
            lastScientificSend = millis();
        }
        
        static unsigned long badPostureStart = 0;
        static bool alertSent = false;
        
        if (postureResult.indexOf("BAD") >= 0) {
            if (badPostureStart == 0) {
                badPostureStart = millis();
            }
            
            if (!alertSent && (millis() - badPostureStart) > alertSettings.bad_posture_duration) {
                String alertMsg = "Bad posture detected for ";
                alertMsg += (millis() - badPostureStart) / 1000;
                alertMsg += " seconds";
                
                bluetooth.print("ALERT:{\"type\":\"POSTURE\",\"message\":\"");
                bluetooth.print(alertMsg);
                bluetooth.println("\"}");
                
                alertSent = true;
            }
        } else {
            badPostureStart = 0;
            alertSent = false;
        }
    }
}
