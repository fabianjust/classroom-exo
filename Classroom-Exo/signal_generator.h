/**
 * @file signal_generator.h
 * @brief Signal generation module for various waveform types
 * @details This module provides functionality to generate different types of signals
 *          including steps, ramps, and periodic waveforms. It's designed to work
 *          with the PID controller generating the input signal to control the servo motor response.
 *
 * @author Sarah Stoiber
 * @date 2024-11-08
 */


#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H


// #include "servo_control.h"
#include "config.h"
#include "servo_control.h"


/**
 * @brief Initialize a signal generator with specified type
 * @param sig Pointer to the SignalGenerator_t structure
 * @param type The type of signal to generate (from SignalType enum)
 * @pre sig must be a valid pointer to an allocated SignalGenerator_t structure
 */
void SignalGenerator_init(SignalGenerator_t* sig, SignalType type);

/**
 * @brief Set the hold time for step or pulse signals
 * @param sig Pointer to the SignalGenerator_t structure
 * @param holdTime Duration to hold each step/pulse value (in milliseconds)
 */
void SignalGenerator_setHoldTime(SignalGenerator_t* sig, unsigned long holdTime);

/**
 * @brief Set the wait time between signal transitions
 * @param sig Pointer to the SignalGenerator_t structure
 * @param waitTime Duration to wait between transitions (in milliseconds)
 */
void SignalGenerator_setWaitTime(SignalGenerator_t* sig, unsigned long waitTime);

/**
 * @brief Configure step values for step response testing
 * @param sig Pointer to the SignalGenerator_t structure
 * @param initialStep Initial step value
 * @param finalStep Final step value
 */
void SignalGenerator_setStepValues(SignalGenerator_t* sig, float initialStep, float finalStep);

/**
 * @brief Set the rate of change for ramp signals
 * @param sig Pointer to the SignalGenerator_t structure
 * @param rate Rate of change (units per second)
 */
void SignalGenerator_setRampRate(SignalGenerator_t* sig, float rate);

/**
 * @brief Set the amplitude for periodic signals
 * @param sig Pointer to the SignalGenerator_t structure
 * @param amp Peak-to-peak amplitude of the signal
 */
void SignalGenerator_setAmplitude(SignalGenerator_t* sig, float amp);

/**
 * @brief Set the frequency for periodic signals
 * @param sig Pointer to the SignalGenerator_t structure
 * @param freq Frequency in Hz
 */
void SignalGenerator_setFrequency(SignalGenerator_t* sig, float freq);

/**
 * @brief Set the range of signal values
 * @param sig Pointer to the SignalGenerator_t structure
 * @param initial Minimum value of the signal
 * @param final Maximum value of the signal
 */
void SignalGenerator_setRange(SignalGenerator_t* sig, float initial, float final);

/**
 * @brief Set the number of iterations for the signal pattern
 * @param sig Pointer to the SignalGenerator_t structure
 * @param iterations Number of times to repeat the signal pattern (0 for infinite)
 */
void SignalGenerator_setIterations(SignalGenerator_t* sig, int iterations);

/**
 * @brief Set maximum duration for signal generation
 * @param sig Pointer to the SignalGenerator_t structure
 * @param duration Maximum duration in milliseconds (0 for no limit)
 */
void SignalGenerator_setMaxDuration(SignalGenerator_t* sig, unsigned long duration);

/**
 * @brief Start signal generation
 * @param sig Pointer to the SignalGenerator_t structure
 */
void SignalGenerator_start(SignalGenerator_t* sig);

/**
 * @brief Stop signal generation
 * @param sig Pointer to the SignalGenerator_t structure
 */
void SignalGenerator_stop(SignalGenerator_t* sig);

/**
 * @brief Reset the signal generator to initial conditions
 * @param sig Pointer to the SignalGenerator_t structure
 */
void SignalGenerator_reset(SignalGenerator_t* sig);

/**
 * @brief End signal generation and perform cleanup
 * @param sig Pointer to the SignalGenerator_t structure
 * @param controller Pointer to associated PID controller
 */
void SignalGenerator_end(SignalGenerator_t* sig, PIDController_t *controller);

/**
 * @brief Generate the next signal value
 * @param sig Pointer to the SignalGenerator_t structure
 * @return The generated signal value
 */
float SignalGenerator_generateSignal(SignalGenerator_t* sig);

/**
 * @brief Update the signal generator state
 * @param sig Pointer to the SignalGenerator_t structure
 * @param controller Pointer to associated PID controller
 * @details This function is called periodically to update the signal
 *          generator's internal state and generate new values
 */
void SignalGenerator_update(SignalGenerator_t* sig, PIDController_t *controller);

/**
 * @brief Advance to the next step in step response testing
 * @param sig Pointer to the SignalGenerator_t structure
 * @param controller Pointer to associated PID controller
 */
void SignalGenerator_moveToNextStep(SignalGenerator_t* sig, PIDController_t *controller);

/**
 * @brief Print current status information
 * @param sig Pointer to the SignalGenerator_t structure
 * @param controller Pointer to associated PID controller
 */
void SignalGenerator_printStatus(SignalGenerator_t* sig, PIDController_t *controller);

/**
 * @brief Check if the signal generator is currently active
 * @param sig Pointer to the SignalGenerator_t structure
 * @return true if the generator is active, false otherwise
 */
bool SignalGenerator_isActive(SignalGenerator_t* sig);

/**
 * @brief Get the current iteration count
 * @param sig Pointer to the SignalGenerator_t structure
 * @return Current iteration number
 */
int SignalGenerator_getCurrentIteration(SignalGenerator_t* sig);

#endif // SIGNAL_GENERATOR_H