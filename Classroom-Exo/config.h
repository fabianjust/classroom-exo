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
#define SERVOMOVEMENT

#define BAUD_RATE 115200  ///< Serial communication baud rate
#define NINA_RESETN 6     ///< Reset pin for NINA module
#define MAX_DATA_LENGTH 200

#include <Arduino.h>
#include <Servo.h>
#include "messaging_codes.h"
#include "control_modes.h"


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
const uint16_t minAngleServoMicros = 2300;

/// @brief Maximum angle servo position in microseconds.
const uint16_t maxAngleServoMicros = 1050;



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
 * @brief Structure for device settings
 */
typedef struct {
    String deviceName;
    uint16_t servoZeroReferenceBytes;
    uint16_t servo140ReferenceBytes;
    uint16_t servoZeroReferenceMicros;
    uint16_t servo140ReferenceMicros;
    uint16_t forceZeroReferenceValue;
} DeviceSettings;

const DeviceSettings deviceSettingsArray[] = {
    // deviceName, servoZeroReferenceBytes, servo140ReferenceBytes, servoZeroReferenceMicros, servo140ReferenceMicros, forceZeroReferenceValue
    {"01_CLASSROOM_EDU_EXO_PRO", 164, 571, 2296, 1059, 379},
    {"02_CLASSROOM_EDU_EXO_PRO", 164, 570, 2296, 1059, 379},
    {"03_CLASSROOM_EDU_EXO_PRO", 160, 570, 2296, 1059, 379},
    {"04_CLASSROOM_EDU_EXO_PRO", 164, 571, 2296, 1059, 350}, // not true!
    {"05_CLASSROOM_EDU_EXO_PRO", 164, 571, 2296, 1059, 379}, // not true!
    {"06_CLASSROOM_EDU_EXO_PRO", 164, 562, 2296, 1059, 340},
    {"07_CLASSROOM_EDU_EXO_PRO", 168, 573, 2296, 1059, 379},
    {"08_CLASSROOM_EDU_EXO_PRO", 162, 568, 2296, 1059, 379},
    {"ESP32-BT-Slave", 100, 100, 1000, 1000, 100}};

/** @brief Global instance of message to send */
extern message msg2send;

/** @brief Global instance of message to receive */
extern message msg2receive;

/** @brief Global instance of current device settings */
extern DeviceSettings currentSettings;


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