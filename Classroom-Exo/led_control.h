/**
 * @file led_control.h
 * @brief Header file for LED Button control
 *
 * This file contains declarations for functions and variables
 * related to controlling the state of the LED  Button.
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "config.h"

/// @brief Time in ms when LED was last changed
extern unsigned long lastChange;

/// @brief Delay in ms before next LED state change
extern unsigned long delayTime;

/// @brief Ensures if-clause is performed immediately on first call
extern boolean firstTime;

/**
 * @brief Initializes the LED Button to bright white
 */
void initializeLED();


/**
 * @brief Controls LED blinking based on system status
 */
 void blinkLED_Button();                           

/**
 * @brief Change the color of the RGB LED button
 *
 * @param colour An 8-bit unsigned integer representing the desired color
 * @note See implementation file for color code details
 */
void changeLEDButtonColor(uint8_t colour);

#endif