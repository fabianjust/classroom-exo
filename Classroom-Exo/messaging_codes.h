
#include <Arduino.h>


// define header and opcodes
const uint8_t HEADER = 0xFF;
const uint8_t EMG_STREAM_HEADER = 0x9F;
const uint8_t FORCE_STREAM_HEADER = 0x8F;
const uint8_t ANGLE_STREAM_HEADER = 0x7F;

const uint8_t TEST_CONNECT = 0x01;
const uint8_t SET_LED = 0x02;
const uint8_t GET_BAT_VOLTAGE = 0x03;
const uint8_t ACTV_SERVO = 0x04;
const uint8_t MOVE_SERVO = 0x05;
const uint8_t GET_SERVO_ANGLE = 0x06;
const uint8_t GET_FORCE = 0x07;
const uint8_t GET_EMG = 0x08;
const uint8_t GET_ALL_SENSORS = 0x09;
const uint8_t RECV_SERVO_POS = 0x10;
const uint8_t SET_SPEED = 0x11;
const uint8_t MOVE_PID = 0x12;
const uint8_t ZERO_SERVO = 0x13;
const uint8_t STOP_CONNECT = 0x15;
const uint8_t SET_PACKAGE_COUNTER = 0x16;
const uint8_t START_PICONTROLLER = 0x17;
const uint8_t SET_PI_CONTROLLER_PARAMS = 0x19;
// const uint8_t STREAM_EMG = 0x22;

const uint8_t CALIBRATE_SERVO = 0x30;
const uint8_t CALIBRATE_EMG = 0x31;
const uint8_t CALIBRATE_FORCE = 0x32;

const uint8_t SET_THRESHOLDS = 0x20;
const uint8_t STOP_AQUISITION = 0x21;
const uint8_t START_AQUISITION = 0x22;
const uint8_t EMERGENCY_STOP = 0x23;
const uint8_t SET_ANALOG_PINS = 0x24;

const uint8_t BI_EMG_STREAM = 0x40;
const uint8_t FORCE_STREAM = 0x41;
const uint8_t MONO_EMG_STREAM = 0x42;
const uint8_t EMG_STREAM_MIN_MAX = 0x43;
const uint8_t IMU_STREAM = 0x44;
const uint8_t EMG_STREAM_INCREMENT = 0x45;


// define answer codes
const uint8_t VERIF_CONNECTION = 0x50;
const uint8_t LED_SET = 0x51;
const uint8_t SERVO_ACTIVATED = 0x52;
const uint8_t SERVO_DEACTIVATED = 0x53;
const uint8_t PID_RECEIVED = 0x54;
const uint8_t SPEED_SET = 0x55;

const uint8_t STATE_ANSWER = 0xAA;
const uint8_t COMMAND_OK = 0xBB;
const uint8_t DATA_ANSWER = 0xCC;
const uint8_t FORCE_ANSWER = 0x8F;
const uint8_t EMG_ANSWER = 0x9F;
const uint8_t SERVO_ANSWER = 0x7F;
const uint8_t IMU_ANSWER = 0x6F;

const uint8_t WRONG_MSG = 0x80;
const uint8_t WRONG_HEADER = 0x81;
const uint8_t WRONG_OPCODE = 0x82;
const uint8_t WRONG_LENGTH = 0x83;
const uint8_t WRONG_CHECKSUM = 0x84;