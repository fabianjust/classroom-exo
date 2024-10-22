/**
 * @file sensor_control.h
 * @brief Header file for sensor control module
 *
 * This file contains declarations for functions and variables
 * related to reading and processing EMG and Force sensor data.
 */

#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include "config.h"
#include <Arduino_LSM6DS3.h>


const uint8_t EMG_WINDOW_SIZE = 5;
const uint8_t FORCE_WINDOW_SIZE = 5;
const uint8_t IMU_WINDOW_SIZE = 1;

/**
 * @brief Structure for sensor data
 */
typedef struct {
    float gyroAvg[3];                  /**< Average gyroscope readings for X, Y, Z axes */
    float accelAvg[3];                 /**< Average accelerometer readings for X, Y, Z axes */
    float emgAvg[2];                   /**< Average EMG readings for two channels */
    float forceAvg;                    /**< Average force sensor reading */
    
    float gyroSum[3];                  /**< Sum of gyroscope readings for averaging */
    float accelSum[3];                 /**< Sum of accelerometer readings for averaging */
    float emgSum[2];                   /**< Sum of EMG readings for averaging */
    float forceSum;                    /**< Sum of force sensor readings for averaging */
    
    float gyro[3][IMU_WINDOW_SIZE];    /**< Raw gyroscope data for X, Y, Z axes */
    float accel[3][IMU_WINDOW_SIZE];   /**< Raw accelerometer data for X, Y, Z axes */
    float emg[2][EMG_WINDOW_SIZE];     /**< Raw EMG data for two channels */
    float force[FORCE_WINDOW_SIZE];    /**< Raw force sensor data */
    
    uint16_t angle;                    /**< Current angle measurement */
    
    int imuIndex;                      /**< Current index for IMU (gyro/accel) data in circular buffer */
    int emgIndex;                      /**< Current index for EMG data in circular buffer */
    int forceIndex;                    /**< Current index for force data in circular buffer */
} sensorData;


/// @brief Upper threshold for force measurements
extern int16_t forceThresholdUpper;

/// @brief Lower threshold for force measurements
extern int16_t forceThresholdLower;

/// @brief Upper threshold for EMG measurements
extern uint16_t emgThresholdUpper;

/// @brief Lower threshold for EMG measurements
extern uint16_t emgThresholdLower;

/// @brief General threshold for EMG measurements
extern uint16_t emgThreshold;

/// @brief Current speed setting for the system
extern uint8_t speed_setting;

/// @brief Step size for angle adjustments
extern uint8_t angle_step;

/// @brief Current control mode of the system
extern uint8_t control_mode;

/** @brief Global instance of current device settings */
extern sensorData mySensors;


/** @brief Signal value from EMG sensor 0 */
extern uint16_t EMG0Signal;
extern uint8_t emg_buffer_ndx;
extern uint16_t EMG0Sum;
extern uint16_t EMG0Buffer[EMG_WINDOW_SIZE];
extern uint16_t EMG0AverageBuffer[EMG_WINDOW_SIZE];
extern uint16_t EMG0Average;

/** @brief Signal value from EMG sensor 1 */
extern uint16_t EMG1Signal;
extern uint8_t emg_buffer_ndx;
extern uint16_t EMG1Sum;
extern uint16_t EMG1Buffer[EMG_WINDOW_SIZE];
extern uint16_t EMG1AverageBuffer[EMG_WINDOW_SIZE];
extern uint16_t EMG1Average;

/** @brief Current force reading */
extern int16_t forceIs;
extern uint8_t force_buffer_ndx;
extern int16_t ForceBuffer[FORCE_WINDOW_SIZE];
extern int16_t ForceAverage;
extern int16_t ForceSum;

/** @brief Current IMU readings */
extern float acc_x, acc_y, acc_z;
extern float gy_x, gy_y, gy_z;
extern unsigned long int_gy_x, int_gy_y, int_gy_z;
extern unsigned long int_acc_x, int_acc_y, int_acc_z;

// initialize variable for reading sensor data with 100hz
extern unsigned long currentSensorTime;
extern unsigned long previousSensorTime;
extern unsigned long sensor_interval;


/**
 * @brief Initialize the sensors
 */
void initializeSensors(sensorData *data);

/**
 * @brief Update MVA from all sensors
 */
void calculateMovingAverage(sensorData *data);

/**
 * @brief Update readings from all sensors
 */
void updateSensors(sensorData *data);

/**
 * @brief Read the force sensor
 * @return The current force sensor reading
 */
float readForceSensor(DeviceSettings *currentSettings);

/**
 * @brief Read an EMG sensor
 * @param analogPin The analog pin number to read from
 * @return The current EMG sensor reading
 */
float readEMGSensor(uint8_t analogPin);

/**
 * @brief Reads the current angle of the servo motor
 * @return The current angle of the servo in degrees as a 16-bit unsigned integer
 */
uint16_t readServoAngle();

/**
 * @brief Implements bi-threshold control for the servo motor using either force or emg sensor data
 * @param sensorAverage The average reading from the sensor
 * @param upper_thresh The upper threshold for force or emg
 * @param lower_thresh The lower threshold for force or emg
 */
void biThresholdControl(float sensorAverage, float upper_thresh, float lower_thresh);

/**
 * @brief Implements mono-threshold control for the servo motor using either force or emg sensor data
 * @param sensorAverage The average reading from the sensor
 * @param thresh The threshold for force or emg
 */
void monoThresholdcontrol(float sensorAverage, float thresh);

/**
 * @brief Implements mono-threshold control for the servo motor using  two emg sensor data
 * @param sensor1Average The average reading from the first sensor
 * @param sensor2Average The average reading from the second sensor
 * @param thresh The threshold for force or emg
 */
void monoThresholdBicontrol(float sensor1Average, float sensor2Average, float thresh);

/**
 * @brief Packages sensor data into a message structure
 * @param data The sensor data to be packaged
 * @param msg2send The message structure to be filled with the packaged data
 * @param opcode The operation code to be included in the message
 */
void packageSensorData(const sensorData& data, message *msgOut, uint8_t opcode);

/**
 * @brief Handles the IMU data stream
 * 
 * @param imu_stream A boolean flag indicating whether the IMU stream should be enabled or disabled
 */
void handleIMUStream(bool imu_stream);

/**
 * @brief Handles the general sensor data stream
 * 
 * @param sensor_stream A boolean flag indicating whether the sensor stream should be enabled or disabled
 * @param control_mode The current control mode of the system
 */
void handleSensorStream(bool sensor_stream, uint8_t control_mode);

#endif // SENSOR_CONTROL_H