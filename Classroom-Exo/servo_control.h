/**
 * @file servo_control.h
 * @brief Header file for servo control module
 *
 * This file contains declarations for functions and variables
 * related to controlling the servo motor.
 */
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

#include "config.h"
#include "sensor_control.h"
#include "battery_management.h"
#include "messaging.h"



/** @brief Current state of the servo (on/off) */
extern boolean servo_state;

/** @brief Desired position for the servo */
extern uint16_t positionDesired;

/** @brief Maximum angle the servo can reach */
extern const uint16_t maxAngle;

/** @brief Minimum angle the servo can reach */
extern const uint16_t minAngle;

/// @brief Reference degree for servo positioning
extern uint16_t ref_degree;

/// @brief Step size for angle adjustments
extern uint16_t angleStep;

/// @brief Angle corresponding to the servo's zero position
extern uint16_t zeroServoAngle;

/// @brief Counter for zeroing/resetting
extern uint8_t zero_counter;

/// @brief Flag indicating whether the servo is in zeroing/resetting mode
extern bool zero_servo;

/// @brief Current timestamp for servo operations
extern unsigned long currentServoTime;

/// @brief Previous timestamp for servo operations
extern unsigned long previousServoTime;

/// @brief Time interval between servo movements
extern unsigned long servo_move_interval;

/// @brief Size of the big buffer for data storage
const uint16_t BIG_BUFFER_SIZE = 1700;

/// @brief Large buffer for temporary data storage
extern uint16_t bigBuffer[];

/// @brief Current index position in the big buffer
extern uint16_t bigBufferIndex;

/// @brief Number of PID packages
extern uint16_t pid_package_nums;

/// @brief Temporary buffer value
extern uint16_t tempbufferval;

/// @brief Counter for PID index
extern uint16_t pid_indx_counter;

/// @brief Counter for PID packages
extern uint16_t pid_package_counter;

/// @brief Counter for servo steps
extern uint16_t servoStepCounter;

/// @brief Step value for servo movement
extern int16_t step;

/// @brief Array storing the PID curve values
extern uint16_t pid_curve[];

/// @brief Number of steps in the PID curve
extern uint16_t PID_steps;

/// @brief Counter for operation cycles
extern uint8_t cycle_counter;

extern float previousError;

// Step response variables
extern bool stepResponseActive;
extern unsigned long stepStartTime;
const unsigned long maxStepDuration = 15000;  // 15 seconds maximum for each step
const float positionTolerance = 50;  // Tolerance for considering position reached (in PWM)
const unsigned long settleDuration = 1000;  // Time to stay within tolerance (in milliseconds)
extern unsigned long withinToleranceTime;  // Time when position first came within tolerance

// New variable to control program start
extern bool programStarted;

// Low-pass filter variables
const float alpha = 0.2; // Filter coefficient (0 < alpha < 1)
extern float filteredPosition; // Initial filtered position (center of 10-bit ADC range)


/**
 * @brief Structure for PI Controller
 */
typedef struct {
    int pwm_target;
    int pwm_current;
    float velocity_current;
    float integral_error;
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float velocity_limit;
    float integral_limit;  // To prevent integral windup
    int deadband;  // Deadband range
    int min_pwm; // Minimum allowed PWM value
    int max_pwm; // Maximum allowed PWM value
    int min_step; // Minimum step size (twice the deadband)
} PIController_t;


/**
 * @brief Structure for step response parameters
 */
typedef struct{
    bool active;
    unsigned long startTime;
    unsigned long maxDuration;
    int positionTolerance;
    unsigned long settleDuration;
    unsigned long withinToleranceTime;
    uint16_t initialStep;
    uint16_t finalStep;
    int currentIteration;
    int totalIterations;
    bool isInitialStep;
} StepResponse_t;

/** @brief Global instance of current device settings */
extern PIController_t servo_controller;

extern StepResponse_t step_response;

/**
 * @brief Initialize the servo
 * @return true if successfull, false otherwise
 */
bool initializeServo();

/**
 * @brief Activate the servo motor.
 * @return set servo_state to true
 */
bool activateServo();

/**
 * @brief Deactivate the servo motor.
 * @return set servo_state to false
 */
bool deactivateServo();

/**
 * @brief Move the servo to a specific position.
 * @param Target position in degrees
 * @return true if sucessfull, falso otherwise
 */
bool moveServo(int position);

/**
 * @brief Converts ADC reading to degrees
 * @param adcread The 10bit ADC value to convert
 * @param defaultsettings Reference to the device settings
 * @return The converted value in degrees
 */
uint16_t adcToDegrees(uint16_t adc_read, const DeviceSettings &defaultsettings);

/**
 * @brief Converts degrees to microseconds
 * @param target_degrees The target position in degrees
 * @param defaultsettings Reference to the device settings
 * @return The converted value in microseconds
 */
uint16_t degreesToMicroseconds(uint16_t target_degrees, const DeviceSettings &defaultsettings);

/**
 * @brief Converts bytes to microseconds
 * @param target_bytes The target value in bytes
 * @param defaultsettings Reference to the device settings
 * @return The converted value in microseconds
 */
uint16_t bytesToMicroseconds(uint16_t target_bytes, const DeviceSettings &defaultsettings);

/**
 * @brief Moves the device to a target position specified in degrees
 * @param target The target position in degreese
 */
void moveToPositionDeg2Microseconds(uint16_t target, const DeviceSettings& settings);

/**
 * @brief Moves the device to a target position specified in microseconds
 * @param target The target position in microseconds
 */
void moveToPositionMicroseconds(uint16_t target, const DeviceSettings& settings);

/**
 * @brief Checks if the given position is valid according to the device settings
 * 
 * @param position The position to be validated
 * @param settings The device settings containing the valid position range
 * @return true If the position is within the valid range, false otherwise
 */
bool isValidPosition(uint16_t position, const DeviceSettings& settings);

/**
 * @brief Handles the calculation and execution of the PID (Proportional-Integral-Derivative) trajectory
 */
void processPIDTrajectory();

/**
 * @brief Updates the servo position based on the control mode and sensor data
 * 
 * @param control_mode The current control mode of the system
 * @param data Pointer to the sensor data structure
 */
void updateServoPosition(uint8_t control_mode, sensorData *data);

/**
 * @brief Packages the current servo data into a message structure
 * 
 * @param msg2send Pointer to the message structure where the servo data will be stored
 * @param currentPosition The current position of the servo
 */
void packageServoData(message *msg2send, uint16_t currentPosition);

/**
 * @brief Handles the logic for resetting the servo to its initial or calibrated position
 */
void processServoResetting();

void pi_handler();

void performStepResponse();

int readAndFilterPosition();

int mapAnalogToPWM();

void startStepResponse();

void PIController_tick(PIController_t *controller, float dt);

bool isPositionReached();

#endif // SERVO_CONTROL_H