/**
 * @file battery_management.h
 * @brief Header file for battery management module
 *
 */

#ifndef BATTERY_MANAGEMENT_H
#define BATTERY_MANAGEMENT_H

#include "config.h"
#include "led_control.h"
#include "messaging.h"

/// @brief Upper limit for high battery voltage warning (in millivolts)
const float HIGH_BAT_LIMIT_WARNING = 7600;

/// @brief Upper limit for medium battery voltage warning (in millivolts)
const float MED_BAT_LIMIT_WARNING = 7200;

/// @brief Upper limit for low battery voltage warning (in millivolts)
const float LOW_BAT_LIMIT_WARNING = 6800;

/// @brief Critical battery voltage limit for warning (in millivolts)
const float BAT_LIMIT_WARNING = 6200;

/// @brief Current battery voltage in millivolts
extern float bat_mvolt;

/// @brief  intervall for polling battery voltage
const uint8_t POLL_INTERVALL = 10000;

/// @brief  window size for moving average filtered battery voltage
const uint8_t BAT_WINDOW_SIZE = 10;

/**
 * @struct BatteryVoltage
 * @brief Stores battery voltage data and calculates moving average.
 */
struct BatteryVoltage {
    float samples[BAT_WINDOW_SIZE]; /**< Array to store voltage samples */
    uint8_t currentIndex;           /**< Current index in the samples array */
    float sum;                      /**< Sum of all samples in the window */
    float batAverag;                /**< Calculated moving average of battery voltage */
    bool isFull;                    /**< Flag indicating if the sample window is full */
    unsigned long lastPollTime;     /**< Timestamp of the last voltage poll */
};

extern BatteryVoltage myBat;

/**
 * @brief Initializes a BatteryVoltage structure.
 *
 * @param data Pointer to the BatteryVoltage structure to initialize.
 */
void initBatteryVoltage(BatteryVoltage *data);

/**
 * @brief Check the current battery voltage and update LED Button status
 * 
 * @return Current battery voltage in millivolts
 */
void pollBatteryVoltage(BatteryVoltage* bat_mvolt);

/**
 * @brief Updates the moving average with a new voltage sample.
 *
 * @param bv Pointer to the BatteryVoltage structure to update.
 * @param newVoltage The new voltage sample to add to the moving average.
 */
void updateMovingAverage(BatteryVoltage *bv, float newVoltage);

/**
 * @brief Read the current battery voltage
 * 
 * @return Current battery voltage in millivolts
 */
float readBatteryVoltage();

#endif // BATTERY_MANAGEMENT_H