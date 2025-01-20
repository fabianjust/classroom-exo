/**
 * @file config.h
 * @brief Configuration header file
 *
 * This file contains global variables and structures used across multiple modules, as well as Pin assignments and constants for the project
 */

#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG
// #define DEBUG2
// #define DEBUG_BAT
#define SERVOMOVEMENT

#define BAUD_RATE 115200  ///< Serial communication baud rate
#define NINA_RESETN 6     ///< Reset pin for NINA module
#define MAX_DATA_LENGTH 200
#define NUM_CALIBRATION_POINTS 12

#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include "messaging_codes.h"
#include "control_modes.h"
#include "numeric_utils.h"


/// @brief Pin number for the onboard LED
const uint8_t LED_PIN = 13;

/// @brief Pin number for the red component of an RGB LED
const uint8_t RED_PIN = 9;

/// @brief Pin number for the green component of an RGB LED
const uint8_t GREEN_PIN = 6;

/// @brief Pin number for the blue component of an RGB LED
const uint8_t BLUE_PIN = 3;

/// @brief Pin number for servo control signal
const uint8_t SERVO_CONTROL_PIN = 5;

/// @brief Pin number for servo power control
const uint8_t SERVO_POWER_PIN = 2;

/// @brief Analog pin number for reading servo position
const uint8_t SERVO_POSITION_PIN = A3;

/// @brief Analog pin number for force sensor input
const uint8_t FORCE_SENSOR_PIN = A0;

/// @brief Analog pin number for the first EMG sensor input
const uint8_t EMG_SENSOR0_PIN = A1;

/// @brief Analog pin number for the second EMG sensor input
const uint8_t EMG_SENSOR1_PIN = A2;

/// @brief Pin number for EMG sensor selection
extern uint8_t EMG0_SELECT_PIN;

/// @brief Pin number for EMG sensor selection
extern uint8_t EMG1_SELECT_PIN;

/// @brief Analog pin number for battery voltage measurement
const uint8_t BATTERY_VOLTAGE_PIN = A6;

/// @brief Baud rate for serial communication
const uint16_t SERIAL_BAUD_RATE = 115200;

/// @brief Total number of CLASSROOM EXO devices
const uint8_t NUM_DEVICES = 9;

/// @brief Minimum angle servo position in microseconds.
const uint16_t minAngleServoMicros = 2400;

/// @brief Maximum angle servo position in microseconds.
const uint16_t maxAngleServoMicros = 1265;



/** @brief Name of the device */
extern String device_name;

/** @brief Flag indicating if the device is okay */
extern bool dev_ok;

extern uint16_t current_servo_position;
extern uint16_t current_servo_micros;

/**
 * @brief Structure for messages
 */
typedef struct {
    uint8_t header;     // intended to be HEADER
    uint8_t opcode;     // intended to be one from the list above
    uint8_t dataLength; // intended to be less than MAX_DATA_LENGTH
    uint8_t data[MAX_DATA_LENGTH];
    // A little-endian system stores the least significant byte (LSB) at the lowest memory address. The “little end” (the least significant part of the data) comes first.
    // msg2send.data[0] = EMG0Average>>8u; // HIGH
    // msg2send.data[1] = EMG0Average; // LOW
    uint8_t checksum; // intended to be the sum of all received bytes
} message;


/**
 * @brief Structure for PI Controller
 */
typedef struct {
    int pwm_target;
    int pwm_current;
    int pwm_calculated;
    double current_angle;
    double setpoint_target;
    double pid_output;
    double pid_input;
    float velocity_current;
    float integral_error;
    double Kp;  // Proportional gain
    double Ki;  // Integral gain
    double Kd;
    float velocity_limit;
    float integral_limit;  // To prevent integral windup
    int deadband;  // Deadband range
    int min_pwm; // Minimum allowed PWM value
    int max_pwm; // Maximum allowed PWM value
    int min_step; // Minimum step size (twice the deadband)
} PIDController_t;


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
    float initialStep;
    float finalStep;
    int currentIteration;
    int totalIterations;
    bool isInitialStep;
    bool isWaiting;
    unsigned long waitStartTime;
    unsigned long waitDuration;
} StepResponse_t;

// Signal type enumeration
enum SignalType {
    STEP,
    RAMP,
    COSINE
};

// Signal generator structure
typedef struct {
    SignalType type;
    bool active;
    unsigned long startTime;
    unsigned long maxDuration;
    unsigned long withinToleranceTime;
    float initialValue;
    float finalValue;
    int currentIteration;
    int totalIterations;
    float rampRate;
    float amplitude;
    float frequency;
    float offset;
    bool isInitialStep;
    bool rampingUp;
    bool holding;
    unsigned long holdTime;
    unsigned long holdStartTime;
    bool rampingDown;    // New state for explicit ramp down
    bool holdingLow;     // New state for holding at bottom
    
    // Added fields for step response with waiting
    bool isWaiting;
    unsigned long waitStartTime;
    unsigned long waitDuration;
    float initialStep;
    float finalStep;
} SignalGenerator_t;


// Calibration point structure
typedef struct {
    uint16_t potValue;
    uint16_t angleX10;
} CalibrationPoint;


/**
 * @brief Structure for device settings
 */
typedef struct {
    String deviceName;
    uint16_t servoZeroReferenceBytes;
    uint16_t servo110ReferenceBytes;
    uint16_t servoZeroReferenceDegrees;
    uint16_t servo110ReferenceDegrees;
    int servoZeroReferenceMicros;
    int servo110ReferenceMicros;
    uint16_t forceZeroReferenceValue;
    CalibrationPoint servoCalibration[NUM_CALIBRATION_POINTS];
} DeviceSettings;

const DeviceSettings deviceSettingsArray[] = {
    // deviceName, servoZeroReferenceBytes, servo110ReferenceBytes, servoZeroReferenceDegrees, servo110ReferenceDegrees, servoZeroReferenceMicros, servo110ReferenceMicros, forceZeroReferenceValue
    {"01_CLASSROOM_EDU_EXO_PRO", 499, 2478, 1800, 700, 2400, 1265, 1495, {  
        {499, 1800},   // 180.0°
        {671, 1700},   // 170.0°
        {855, 1600},   // 160.0°
        {1026, 1500},   // 150.0°
        {1212, 1400},  // 140.0°
        {1384, 1300},  // 130.0°
        {1562, 1200},  // 120.0°
        {1747, 1100},  // 110.0°
        {1925, 1000},  // 100.0°
        {2115, 900},   // 90.0°
        {2291, 800},   // 80.0°
        {2478, 700}    // 70.0°
    }}, 
    {"02_CLASSROOM_EDU_EXO_PRO", 464, 2438, 1800, 700, 2400, 1265, 1500, { 
        {471, 1800},   // 180.0°
        {633, 1700},   // 170.0°
        {833, 1600},   // 160.0°
        {1007, 1500},   // 150.0°
        {1176, 1400},  // 140.0°
        {1377, 1300},  // 130.0°
        {1573, 1200},  // 120.0°
        {1755, 1100},  // 110.0°
        {1907, 1000},  // 100.0°
        {2082, 900},   // 90.0°
        {2277, 800},   // 80.0°
        {2438, 700}    // 70.0°
  }}, 
    {"03_CLASSROOM_EDU_EXO_PRO", 457, 2462, 1800, 700, 2296, 1059, 1470, { 
        {457, 1800},   // 180.0°
        {633, 1700},   // 170.0°
        {817, 1600},   // 160.0°
        {1004, 1500},   // 150.0°
        {1192, 1400},  // 140.0°
        {1367, 1300},  // 130.0°
        {1550, 1200},  // 120.0°
        {1742, 1100},  // 110.0°
        {1923, 1000},  // 100.0°
        {2096, 900},   // 90.0°
        {2281, 800},   // 80.0°
        {2462, 700}    // 70.0°
    }},
    {"04_CLASSROOM_EDU_EXO_PRO", 501, 2465, 1800, 700, 2400, 1265, 1470, { 
        {501, 1800},   // 180.0°
        {655, 1700},   // 170.0°
        {836, 1600},   // 160.0°
        {1009, 1500},   // 150.0°
        {1200, 1400},  // 140.0°
        {1374, 1300},  // 130.0°
        {1553, 1200},  // 120.0°
        {1749, 1100},  // 110.0°
        {1928, 1000},  // 100.0°
        {2093, 900},   // 90.0°
        {2297, 800},   // 80.0°
        {2465, 700}    // 70.0°
    }},
    {"05_CLASSROOM_EDU_EXO_PRO", 863, 308, 1800, 700, 2296, 1059, 1500, { // not true!
        {3621, 1800},   // 180.0°
        {3429, 1700},   // 170.0°
        {3250, 1600},   // 160.0°
        {3062, 1500},   // 150.0°
        {2893, 1400},  // 140.0°
        {2705, 1300},  // 130.0°
        {2532, 1200},  // 120.0°
        {2337, 1100},  // 110.0°
        {2147, 1000},  // 100.0°
        {1970, 900},   // 90.0°
        {1792, 800},   // 80.0°
        {1616, 700}    // 70.0°
    }}, // not true!
    {"06_CLASSROOM_EDU_EXO_PRO", 436, 2451, 1800, 700, 2400, 1265, 1370, { 
        {436, 1800},   // 180.0°
        {636, 1700},   // 170.0°
        {807, 1600},   // 160.0°
        {995, 1500},   // 150.0°
        {1172, 1400},  // 140.0°
        {1355, 1300},  // 130.0°
        {1523, 1200},  // 120.0°
        {1699, 1100},  // 110.0°
        {1886, 1000},  // 100.0°
        {2077, 900},   // 90.0°
        {2263, 800},   // 80.0°
        {2451, 700}    // 70.0°
    }},
    {"07_CLASSROOM_EDU_EXO_PRO", 461, 2460, 1800, 700, 2400, 1265, 1406, { 
        {455, 1800},   // 180.0°
        {637, 1700},   // 170.0°
        {820, 1600},   // 160.0°
        {993, 1500},   // 150.0°
        {1174, 1400},  // 140.0°
        {1344, 1300},  // 130.0°
        {1541, 1200},  // 120.0°
        {1722, 1100},  // 110.0°
        {1899, 1000},  // 100.0°
        {2083, 900},   // 90.0°
        {2267, 800},   // 80.0°
        {2457, 700}    // 70.0°
    }},
    {"08_CLASSROOM_EDU_EXO_PRO", 486, 2458, 1800, 700, 2400, 1265, 1700, { 
        {486, 1800},   // 180.0°
        {653, 1700},   // 170.0°
        {841, 1600},   // 160.0°
        {1004, 1500},   // 150.0°
        {1198, 1400},  // 140.0°
        {1383, 1300},  // 130.0°
        {1566, 1200},  // 120.0°
        {1733, 1100},  // 110.0°
        {1910, 1000},  // 100.0°
        {2102, 900},   // 90.0°
        {2277, 800},   // 80.0°
        {2458, 700}    // 70.0°
    }}, 
    {"ESP32-BT-Slave", 1000, 1000, 1000, 1000, 1000, 1000, 1400, {
        {100, 1800},   // 180.0°
        {200, 1700},   // 170.0°
        {300, 1600},   // 160.0°
        {400, 1500},   // 150.0°
        {500, 1400},  // 140.0°
        {600, 1300},  // 130.0°
        {700, 1200},  // 120.0°
        {800, 1100},  // 110.0°
        {900, 1000},  // 100.0°
        {1000, 900},   // 90.0°
        {1100, 800},   // 80.0°
        {1200, 700}    // 70.0°
    }}};


/** @brief Global instance of message to send */
extern message msg2send;

/** @brief Global instance of message to receive */
extern message msg2receive;

/** @brief Global instance of current device settings */
extern DeviceSettings currentSettings;

/** @brief Global instance of current device settings */
extern PIDController_t servo_controller;

extern StepResponse_t step_response;

// Global variables
extern SignalGenerator_t signalGen;  // Declaration of global variable

/** 
 * @brief Initialize the basic hardware components. 
 */
void initializeHardware();

/**
 * @brief Finds device settings based on the given name
 * 
 * @param name The name of the device settings to find
 * @param settings Reference to a DeviceSettings object where the found settings will be stored
 * @return true If the settings were found successfully, false otherwise
 */
bool findDeviceSettings(const String& name, DeviceSettings *settings);

/**
 * @brief Checks the name of the device, and saves default values accordingly
 */
void checkDeviceIntegrity();
#endif // CONFIG_H