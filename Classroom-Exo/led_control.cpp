/**
 * @file led_control.cpp
 * @brief LED color control functions for a RGB LED button
 *
 * This file contains functions to control the color of an RGB LED button.
 * It uses PWM (analogWrite) to set different colors based on input codes.
 */

#include "led_control.h"
#include "messaging.h"

/// @brief Time in ms when LED was last changed
unsigned long lastChange = 0;

/// @brief Delay in ms before next LED state change
unsigned long delayTime = 300;

/// @brief Ensures if-clause is performed immediately on first call
boolean firstTime = true;

void initializeLED() {
  changeLEDButtonColor(0x10);
  #ifdef DEBUG
  Serial.println("LED set to 0x01: Bright White");
  #endif
}

/**
 * @brief Change the color of the LED button
 *
 * This function sets the color of an RGB LED button based on the input color code.
 * It uses PWM to control the brightness of each LED component (Red, Green, Blue).
 *
 * @param colour An 8-bit unsigned integer representing the desired color
 *               - 0x01: Bright White
 *               - 0x02: Violet
 *               - 0x03: Green
 *               - 0x04: Blue
 *               - 0x05: Red
 *               - 0x06: Yellow
 *               - 0x07: Orange
 *               - 0x08: Light Green
 *               - 0x09: Light Blue
 *               - 0x10: Pinkish
 *               - Any other value: Off
 *
 * @note LED Power Button Color Codes:
 *       255 -> LOW, LED is off (R, G, B)
 *       0 -> HIGH, LED is on full brightness (R, G, B)
 */
void changeLEDButtonColor(uint8_t colour) {
    switch(colour) {
        case 0x01: // bright white
            analogWrite(RED_PIN, 100);
            analogWrite(GREEN_PIN, 0);
            analogWrite(BLUE_PIN, 0);
            break;

        case 0x02: // violet
            analogWrite(RED_PIN, 175);
            analogWrite(GREEN_PIN, 255);
            analogWrite(BLUE_PIN, 0);
            break;

        case 0x03:  // green
            analogWrite(RED_PIN, 255);
            analogWrite(GREEN_PIN, 0);
            analogWrite(BLUE_PIN, 255);
            break;
  
        case 0x04:  // blue
            analogWrite(RED_PIN, 255);
            analogWrite(GREEN_PIN, 255);
            analogWrite(BLUE_PIN, 0);
            break;

        case 0x05: // red
            analogWrite(RED_PIN, 0);
            analogWrite(GREEN_PIN, 255);
            analogWrite(BLUE_PIN, 255);
            break;

        case 0x06: // yellow
            analogWrite(RED_PIN, 0);
            analogWrite(GREEN_PIN, 0);
            analogWrite(BLUE_PIN, 255);
            break;
  
        case 0x07: // orange
            analogWrite(RED_PIN, 0);
            analogWrite(GREEN_PIN, 65);
            analogWrite(BLUE_PIN, 255);
            break;

        case 0x08: // light green
            analogWrite(RED_PIN, 235);
            analogWrite(GREEN_PIN, 0);
            analogWrite(BLUE_PIN, 235);
            break;

        case 0x09: // light blue
            analogWrite(RED_PIN, 255);
            analogWrite(GREEN_PIN, 120);
            analogWrite(BLUE_PIN, 140);
            break;

        case 0x10: // pinkish
            analogWrite(RED_PIN, 0);
            analogWrite(GREEN_PIN, 200);
            analogWrite(BLUE_PIN, 200);
            break;

        default: // off
            analogWrite(RED_PIN, 255);
            analogWrite(GREEN_PIN, 255);
            analogWrite(BLUE_PIN, 255);
            break;
    }
}


/**
 * @brief Controls LED blinking based on system status
 * 
 * This function manages the blinking of LEDs to indicate different system states.
 * It uses millis() for non-blocking timing and controls multiple LED colors.
 * 
 * @pre Global variables lastChange, delayTime, firstTime, MatlabConnected, 
 *      EmergencyStop, LED_PIN, BLUE_PIN, RED_PIN, GREEN_PIN must be defined.
 * 
 * @note This function should be called regularly in the main loop.
 */
void blinkLED_Button()
{
    unsigned long currentTime = millis();
    boolean ledState = digitalRead(LED_PIN);
    boolean timeToChange = (currentTime - lastChange >= delayTime) || firstTime;

    if (timeToChange && !MatlabConnected)
    {
        lastChange = currentTime;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);

        if (EmergencyStop)
        {
            // Red blinking in emergency stop
            analogWrite(RED_PIN, ledState ? 0 : 255);
            analogWrite(GREEN_PIN, 255);
            analogWrite(BLUE_PIN, 255);
        }
        else
        {
            // Blue blinking in normal operation
            analogWrite(BLUE_PIN, ledState ? 0 : 255);
            analogWrite(RED_PIN, 255);
            analogWrite(GREEN_PIN, 255);
        }

        firstTime = false;
    }
}