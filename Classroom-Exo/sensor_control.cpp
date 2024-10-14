
#include "sensor_control.h"
#include "servo_control.h"
#include "messaging.h"


unsigned long currentSensorTime = 0;      // stores the current time
unsigned long previousSensorTime = 0;     // Stores the last time the message was sent
unsigned long sensor_interval = 20; // 20 ms interval


void initializeSensors(sensorData* data) {
    if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
    }

    memset(data, 0, sizeof(sensorData));
    #ifdef DEBUG
    Serial.print("Sensors initialized: ");
    Serial.println(data->gyroAvg[0]);
    #endif
}

void updateSensors(sensorData *data) {
    // IMU data
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
        float gy_x, gy_y, gy_z, acc_x, acc_y, acc_z;
        IMU.readGyroscope(gy_x, gy_y, gy_z);
        IMU.readAcceleration(acc_x, acc_y, acc_z);

        float newGyro[3] = {gy_x, gy_y, gy_z};
        float newAccel[3] = {acc_x, acc_y, acc_z};

        for (int i = 0; i < 3; i++) {
            data->gyroSum[i] += newGyro[i] - data->gyro[i][data->imuIndex];
            data->accelSum[i] += newAccel[i] - data->accel[i][data->imuIndex];
            
            data->gyro[i][data->imuIndex] = newGyro[i];
            data->accel[i][data->imuIndex] = newAccel[i];
        }
        // Move to the next index
        data->imuIndex = (data->imuIndex + 1) % IMU_WINDOW_SIZE;


    // EMG data
        float emg0 = readEMGSensor(EMG0_SELECT_PIN);
        float emg1 = readEMGSensor(EMG1_SELECT_PIN);
        float newEmg[2] = {emg0, emg1};

        for (int i = 0; i < 2; i++) {
            data->emgSum[i] += newEmg[i] - data->emg[i][data->emgIndex];
            data->emg[i][data->emgIndex] = newEmg[i];
        }
        // Move to the next index
        data->emgIndex = (data->emgIndex + 1) % EMG_WINDOW_SIZE;

    // Force data
        float newForce = readForceSensor(&currentSettings);
  
        data->forceSum += newForce - data->force[data->forceIndex];
        data->force[data->forceIndex] = newForce;
        // Move to the next index
        data->forceIndex = (data->forceIndex + 1) % FORCE_WINDOW_SIZE;   

    // Angle Sensor
        data->angle = adcToDegrees(readServoAngle(), currentSettings);
    }
    // #ifdef DEBUG
    // Serial.println("Sensors updated");
    // #endif

    calculateMovingAverage(&mySensors);
}

void calculateMovingAverage(sensorData *data) {
  for (int i = 0; i < 3; i++) {
    data->gyroAvg[i] = data->gyroSum[i] / IMU_WINDOW_SIZE;
    data->accelAvg[i] = data->accelSum[i] / IMU_WINDOW_SIZE;
  }

  for (int i = 0; i < 2; i++) {
    data->emgAvg[i] = data->emgSum[i] / EMG_WINDOW_SIZE;
  }

  data->forceAvg = data->forceSum / FORCE_WINDOW_SIZE;
}

float readForceSensor(DeviceSettings *currentSettings) {
    return analogRead(FORCE_SENSOR_PIN)  - currentSettings->forceZeroReferenceValue;
}

float readEMGSensor(uint8_t analogPin){
    return analogRead(analogPin);
}


uint16_t readServoAngle()
{
  return analogRead(SERVO_POSITION_PIN);
}

void biThresholdControl(float sensorAverage, float upper_thresh, float lower_thresh) {

    if (sensorAverage < lower_thresh)
    {
        positionDesired = min(positionDesired + angleStep/speed_setting, minAngle);
    }
    else if (sensorAverage > upper_thresh)
    {
        positionDesired = max(positionDesired - angleStep/speed_setting, maxAngle);
    }

    
    moveToPositionDeg2Microseconds(positionDesired, currentSettings);
    uint16_t positionIs = analogRead(SERVO_POSITION_PIN);
    uint16_t currentServoPosition = adcToDegrees(positionIs, currentSettings);
    

    #ifdef DEBUG
    Serial.print(sensorAverage);
    Serial.print("  ");
    Serial.println(positionDesired);
    #endif

};

void monoThresholdcontrol(float sensorAverage, float thresh) {
    if (sensorAverage < thresh)
    {
        positionDesired = min(positionDesired + angleStep/speed_setting, minAngle);
    }
    else if (sensorAverage >= thresh)
    {
        positionDesired = max(positionDesired - angleStep/speed_setting, maxAngle);
    }

    
    moveToPositionDeg2Microseconds(positionDesired, currentSettings);
    uint16_t positionIs = analogRead(SERVO_POSITION_PIN);
    uint16_t currentServoPosition = adcToDegrees(positionIs, currentSettings);
    

    #ifdef DEBUG
    Serial.print(sensorAverage);
    Serial.print("  ");
    Serial.println(positionDesired);
    #endif
};

void monoThresholdBicontrol(float sensor1Average, float sensor2Average, float thresh) {
    if (sensor1Average < thresh)
    {
        positionDesired = min(positionDesired + angleStep/speed_setting, minAngle);
    }
    else if (sensor2Average >= thresh)
    {
        positionDesired = max(positionDesired - angleStep/speed_setting, maxAngle);
    }

    
    moveToPositionDeg2Microseconds(positionDesired, currentSettings);
    uint16_t positionIs = analogRead(SERVO_POSITION_PIN);
    uint16_t currentServoPosition = adcToDegrees(positionIs, currentSettings);
    

    #ifdef DEBUG
    Serial.print(sensor1Average);
    Serial.print("  ");
    Serial.print(sensor2Average);
    Serial.print("  ");
    Serial.println(positionDesired);
    #endif
};

void handleSensorStream(bool sensor_stream, uint8_t control_mode)
{
    if (!sensor_stream) return;

    unsigned long currentTime = millis();

    // Handle sensor updates
    if (currentTime - previousSensorTime >= sensor_interval)
    {
        previousSensorTime = currentTime;
        updateSensors(&mySensors);
    }

    // Handle servo movement
    if (currentTime - previousServoTime >= servo_move_interval)
    {
        previousServoTime = currentTime;
        updateServoPosition(control_mode, &mySensors);
    }

    // Handle data sending
    if (currentTime - previousSendingTime >= sending_interval)
    {

        Serial.print("Elapsed time: ");
        Serial.println(currentTime - previousSendingTime);
        previousSendingTime = currentTime; // Update the previous time
        packageSensorData(mySensors, &msg2send, control_mode);
        sendMessage(&msg2send);
        Serial.print(control_mode);
        Serial.println(" .. sent Sensor data");
    }
}


void handleIMUStream(bool imu_stream)
{
    if (!imu_stream) return;

    unsigned long currentTime = millis();

    // Update Sensor Data
    if (currentTime - previousSensorTime >= sensor_interval)
    {
        previousSensorTime = currentTime;
        updateSensors(&mySensors);
    }

    // Handle data sending
    if (currentTime - previousSendingTime >= sending_interval)
    {
        Serial.print("Elapsed time: ");
        Serial.println(currentTime - previousSendingTime);
        previousSendingTime = currentTime; // Update the previous time
        packageSensorData(mySensors, &msg2send, IMU_ANSWER);
        sendMessage(&msg2send);
        Serial.println("sent IMU data");
    }
}


void packageSensorData(const sensorData& data, message *msgOut, uint8_t opcode_mode){
    
    msgOut->opcode = opcode_mode;
    msgOut->dataLength = 36+2;  // 4 bytes each for force, 2 emg and 3 gyro and 3 accel values, plus 2bytes for angle

    Serial.println("x  y  z");
    Serial.print(data.gyroAvg[0]); Serial.print(" ");
    Serial.print(data.gyroAvg[2]); Serial.print(" ");
    Serial.print(data.gyroAvg[3]); Serial.print(" ");

    // Use memcpy to copy the bytes of each float into the data array
    memcpy(&msgOut->data[0], &data.forceAvg, sizeof(float));
    memcpy(&msgOut->data[4], &data.emgAvg[0], sizeof(float));
    memcpy(&msgOut->data[8], &data.emgAvg[1], sizeof(float));
    memcpy(&msgOut->data[12], &data.gyroAvg[0], sizeof(float));
    memcpy(&msgOut->data[16], &data.gyroAvg[1], sizeof(float));
    memcpy(&msgOut->data[20], &data.gyroAvg[2], sizeof(float));
    memcpy(&msgOut->data[24], &data.accelAvg[0], sizeof(float));
    memcpy(&msgOut->data[28], &data.accelAvg[1], sizeof(float));
    memcpy(&msgOut->data[32], &data.accelAvg[2], sizeof(float));
    memcpy(&msgOut->data[36], &data.angle, sizeof(uint16_t));
}

