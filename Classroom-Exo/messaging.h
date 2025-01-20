/**
 * @file messaging.h
 * @brief Header file for messaging module
 *
 * This file contains declarations for functions and variables
 * related to communication between the Arduino and Matlab.
 */

#ifndef MESSAGING_H
#define MESSAGING_H

#include "config.h"
#include "sensor_control.h"
#include "battery_management.h"
#include "servo_control.h"
#include "control_modes.h"
#include "signal_generator.h"

/// @brief Flag indicating if Matlab is connected
extern boolean MatlabConnected;

/// @brief Flag indicating Emergency Stop was initiated
extern boolean EmergencyStop;

/// @brief Flag indicating if a new message has been parsed
extern boolean newMessage;

/// @brief Flag indicating if new data has been received
extern boolean newData;

/// @brief Start marker for message framing
extern const byte startMarker;

/// @brief End marker for message framing
extern const byte endMarker;

/// @brief Index for byte-wise operations
extern uint8_t byteIndex;

/// @brief Flag indicating whether data reception is in progress
extern boolean recvInProgress;

/// @brief Length of the data package being received
extern uint8_t packageLength;

/// @brief Constant defining the number of bytes in a package
extern const byte numBytes;

/// @brief Array to store received bytes
extern byte receivedBytes[];

/// @brief Number of bytes received in the current message
extern uint8_t numReceived;

/// @brief Current timestamp for data sending operations
extern unsigned long currentSendingTime;

/// @brief Previous timestamp for data sending operations
extern unsigned long previousSendingTime;

/// @brief Time interval between data sending operations
extern unsigned long sending_interval;

/**
 * @brief Initialize the communication module
 */
void initializeCommunication();

/**
 * @brief get the device name and calibrated values from the device settings
 */
void getDeviceName();

/**
 * @brief Receive bytes from the serial connection
 */
void receiveMessage();

/**
 * @brief Receives a large buffer of data for the pid controller curve
 * @param msgIn Pointer to the message structure containing the incoming data
 */
void receiveBigBuffer(message *msgIn);

/**
 * @brief Reply to a received command
 * @param msgOut Pointer to the message structure to be sent
 */
void sendMessage(message *msgOut);

/**
 * @brief Checks the validity of a received command
 * @param msgIn Pointer to the message structure containing the received command
 * @return true If the command is valid, false otherwise
 */
bool checkReceivedCommand(message *msgIn);

/**
 * @brief Changes modes based on received message and prepares a response
 * @param msgIn Pointer to the incoming message structure
 * @param msgOut Pointer to the outgoing message structure for the response
 */
void changeModes(message *msgIn, message *msgOut);

/**
 * @brief Stores a single byte in a large buffer
 * @param data The byte to be stored in the buffer
 */
void storeByteInBigBuffer(byte data);

/**
 * @brief Stores the pid trajectory in a large buffer, before converting the value to angles
 * @param data The byte to be stored in the buffer
 */
bool checkReceivedCommand(message *msgIn, message *msgOut);

float bytesToFloatFromArray(byte* array, int startIndex);

unsigned long bytesToLongFromArray(byte* array, int startIndex);
#endif  // MESSAGING_H
