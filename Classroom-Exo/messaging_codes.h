/**
 * @file message_codes.h
 * @brief Communication protocol definitions for Arduino-based servo control system
 * @details Contains all message codes used in the communication protocol between
 *          the device and host, including command codes, stream identifiers,
 *          and response codes.
 * 
 * @author Sarah Stoiber
 * @date 2024-11-08
 */



#include <Arduino.h>


/** @name Protocol Headers
 *  @brief Message headers used to identify different types of data packets
 *  @{
 */
const uint8_t HEADER = 0xFF;              ///< Main protocol header
const uint8_t SENSOR_STREAM_HEADER = 0x9F;   ///< sensor data stream header
const uint8_t ANGLE_STREAM_HEADER = 0x8F; ///< Angle sensor stream header
/** @} */

/** @name Basic Commands (0x01-0x09)
 *  @brief Basic device operations and single sensor readings
 *  @{
 */
const uint8_t CMD_TEST_CONNECT = 0x01;        ///< Test connection with device
const uint8_t CMD_SET_LED = 0x02;             ///< Control LED state
const uint8_t CMD_GET_BAT_VOLTAGE = 0x03;     ///< Request battery voltage
const uint8_t CMD_ACTV_SERVO = 0x04;          ///< Activate servo motor
const uint8_t CMD_MOVE_SERVO = 0x05;          ///< Move servo to position
const uint8_t CMD_GET_SERVO_ANGLE = 0x06;     ///< Request current servo angle
const uint8_t CMD_GET_FORCE = 0x07;           ///< Request force sensor reading
const uint8_t CMD_GET_EMG = 0x08;             ///< Request EMG sensor reading
const uint8_t CMD_GET_ALL_SENSORS = 0x09;     ///< Request all sensor readings
/** @} */


/** @name Servo and Control Commands (0x10-0x19)
 *  @brief Extended servo control and system parameters
 *  @{
 */
const uint8_t CMD_RECV_SERVO_POS = 0x10;      ///< Receive servo position
const uint8_t CMD_SET_SPEED = 0x11;           ///< Set servo movement speed
const uint8_t CMD_MOVE_PID = 0x12;            ///< Execute PID-controlled movement
const uint8_t CMD_ZERO_SERVO = 0x13;          ///< Set servo to zero position
const uint8_t CMD_STOP_CONNECT = 0x15;        ///< Terminate connection
const uint8_t CMD_SET_PACKAGE_COUNTER = 0x16; ///< Set package counter value
const uint8_t CMD_START_PICONTROLLER = 0x17;  ///< Start PI controller
const uint8_t CMD_SET_INPUT_RESPONSE_PARAMS = 0x18;  ///< Set input response parameters
const uint8_t CMD_SET_PI_CONTROLLER_PARAMS = 0x19;   ///< Set PI controller parameters
/** @} */


/** @name System Configuration (0x20-0x24)
 *  @brief System settings and acquisition control
 *  @{
 */
const uint8_t CMD_SET_THRESHOLDS = 0x20;      ///< Set sensor thresholds
const uint8_t CMD_STOP_AQUISITION = 0x21;     ///< Stop data acquisition
const uint8_t CMD_START_AQUISITION = 0x22;    ///< Start data acquisition
const uint8_t CMD_EMERGENCY_STOP = 0x23;      ///< Emergency stop all operations
const uint8_t CMD_SET_ANALOG_PINS = 0x24;     ///< Configure analog pin settings
/** @} */

/** @name Calibration Commands (0x30-0x32)
 *  @brief Sensor and system calibration
 *  @{
 */
const uint8_t CMD_CALIBRATE_SERVO = 0x30;     ///< Calibrate servo motor
const uint8_t CMD_CALIBRATE_EMG = 0x31;       ///< Calibrate EMG sensor
const uint8_t CMD_CALIBRATE_FORCE = 0x32;     ///< Calibrate force sensor
/** @} */


/** @name Stream Commands (0x40-0x45)
 *  @brief Continuous data streaming controls
 *  @{
 */
const uint8_t CMD_BI_EMG_STREAM = 0x42;       ///< Stream bilateral EMG data
const uint8_t CMD_FORCE_STREAM = 0x41;        ///< Stream force sensor data
const uint8_t CMD_MONO_EMG_STREAM = 0x40;     ///< Stream single-channel EMG data
const uint8_t CMD_MONO_BI_EMG_STREAM = 0x43;  ///< Stream bilateral EMG data
const uint8_t CMD_IMU_STREAM = 0x44;          ///< Stream IMU data
const uint8_t CMD_EMG_STREAM_INCREMENT = 0x45; ///< Stream EMG with increment
/** @} */


/** @name Acknowledgment Responses (0x50-0x56)
 *  @brief Positive acknowledgment codes
 *  @{
 */
const uint8_t RESP_VERIF_CONNECTION = 0x50;    ///< Connection verified
const uint8_t RESP_LED_SET = 0x51;             ///< LED state changed
const uint8_t RESP_SERVO_ACTIVATED = 0x52;     ///< Servo activation confirmed
const uint8_t RESP_SERVO_DEACTIVATED = 0x53;   ///< Servo deactivation confirmed
const uint8_t RESP_PID_RECEIVED = 0x54;        ///< PID parameters received
const uint8_t RESP_SPEED_SET = 0x55;           ///< Speed setting confirmed
const uint8_t RESP_PID_FINISHED = 0x56;        ///< PID operation completed
/** @} */

/** @name Special Commands
 *  @brief Commands with special purposes
 *  @{
 */
const uint8_t CMD_SET_COSINE_PARAMS = 0x60;   ///< Set cosine wave parameters
/** @} */

/** @name General Status Responses (0xAA-0xCC)
 *  @brief General purpose response codes
 *  @{
 */
const uint8_t RESP_STATE_ANSWER = 0xAA;        ///< State information response
const uint8_t RESP_COMMAND_OK = 0xBB;          ///< Command successfully executed
const uint8_t RESP_DATA_ANSWER = 0xCC;         ///< Generic data response
/** @} */

/** @name Data Response Codes (0x6F-0x9F)
 *  @brief Data packet identifiers
 *  @{
 */
const uint8_t IMU_ANSWER = 0x6F;          ///< IMU data response
const uint8_t SERVO_ANSWER = 0x7F;        ///< Servo position response
const uint8_t FORCE_ANSWER = 0x8F;        ///< Force sensor data response
const uint8_t EMG_ANSWER = 0x9F;          ///< EMG sensor data response
/** @} */

/** @name Error Codes (0x80-0x84)
 *  @brief Communication and protocol error codes
 *  @{
 */
const uint8_t ERR_WRONG_MSG = 0x80;           ///< Invalid message format
const uint8_t ERR_WRONG_HEADER = 0x81;        ///< Invalid message header
const uint8_t ERR_WRONG_OPCODE = 0x82;        ///< Invalid operation code
const uint8_t ERR_WRONG_LENGTH = 0x83;        ///< Invalid message length
const uint8_t ERR_WRONG_CHECKSUM = 0x84;      ///< Checksum verification failed
/** @} */