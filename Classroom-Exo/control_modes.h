/**
 * @file control_modes.h
 * @brief Declarations for control operation modes and streaming flags
 */

#ifndef CONTROL_MODES_H
#define CONTROL_MODES_H

#include <Arduino.h>

/// @brief Flag indicating whether force sensor is in zeroing/resetting mode
extern bool zero_force;

/// @brief Flag indicating whether PID data streaming is active
extern bool pid_stream;

/// @brief Flag indicating whether PID data streaming is active
extern bool PI_programStarted;

/// @brief Flag indicating whether EMG data streaming is active
extern bool emg_stream;

/// @brief Flag indicating whether force sensor data streaming is active
extern bool force_stream;

/// @brief Flag for bi-directional EMG streaming with min-max values
extern bool bi_emg_stream_min_max;

/// @brief Flag for bi-directional EMG streaming with incremental values
extern bool bi_emg_stream_increment;

/// @brief Flag for EMG streaming with incremental values
extern bool emg_stream_increment;

/// @brief Flag for EMG streaming with min-max values
extern bool emg_stream_min_max;

/// @brief Flag indicating whether IMU data streaming is active
extern bool imu_stream;

/// @brief Flag indicating whether general sensor data streaming is active
extern bool sensor_stream;

/// @brief Mode constant for bi-sensor EMG operation
const uint8_t BI_EMG_MODE = 0x42;

/// @brief Mode constant for force sensor operation
const uint8_t FORCE_MODE = 0x41;

/// @brief Mode constant for proportional force sensor operation
const uint8_t PROP_FORCE_MODE = 0x43;

/// @brief Mode constant for mono-sensor EMG operation
const uint8_t MONO_EMG_MODE = 0x40;

/// @brief Mode constant for IMU operation
const uint8_t IMU_MODE = 0x44;

#endif // CONTROL_MODES_H