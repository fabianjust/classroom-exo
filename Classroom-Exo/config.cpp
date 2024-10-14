
#include "config.h"

// initialize message structs
message msg2send;
message msg2receive;

DeviceSettings currentSettings;

uint16_t current_servo_position = 1700;
uint16_t current_servo_micros = 2150;

String device_name = "";
bool dev_ok = false;
bool name_set = false;

uint8_t EMG0_SELECT_PIN = EMG_SENSOR0_PIN;
uint8_t EMG1_SELECT_PIN = EMG_SENSOR1_PIN;

// thresholds
int16_t forceThresholdUpper = 30;
int16_t forceThresholdLower = -30;
uint16_t emgThresholdUpper = 30;
uint16_t emgThresholdLower = 30;
uint16_t emgThreshold = 500;
uint8_t speed_setting = 1;
uint8_t angle_step = 10;

/**
 * @brief Initializes the hardware GPIO pins
 */
void initializeHardware() {
    // Set up LED Pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    // Set up Sensor Pins
    pinMode(EMG_SENSOR0_PIN, INPUT);
    pinMode(EMG_SENSOR1_PIN, INPUT);
    pinMode(FORCE_SENSOR_PIN, INPUT);

    // Set up Servo Motor Pins
    pinMode(SERVO_POSITION_PIN, INPUT);
    // pinMode(SERVO_CONTROL_PIN, OUTPUT);  // not necessary, since that is handled in the servo library!
    pinMode(SERVO_POWER_PIN, OUTPUT);
    
    #ifdef DEBUG
    Serial.println("GPIO Pins initialized  ");
    #endif

}

bool findDeviceSettings(const String& name, DeviceSettings& settings) {
  for (int i = 0; i < NUM_DEVICES; ++i) {
    if (name.equals(deviceSettingsArray[i].deviceName)) {
      settings = deviceSettingsArray[i];
      return true;
    }
  }
  return false;
}

void checkDeviceIntegrity() {
    if (dev_ok & !name_set) {
    Serial.println(device_name);
    delay(50);
    // Lookup the settings for the received device name
    if (findDeviceSettings(device_name, currentSettings)) {
#ifdef DEBUG
      Serial.print("Received device name: ");
      Serial.println(currentSettings.deviceName);
      Serial.println(device_name);
      Serial.print("Zero Servo Reference Bytes: ");
      Serial.println(currentSettings.servoZeroReferenceBytes);
      Serial.print("Ninety Servo Reference Bytes: ");
      Serial.println(currentSettings.servo140ReferenceBytes);
      Serial.print("Force Sensor Zero Reference: ");
      Serial.println(currentSettings.forceZeroReferenceValue);
#endif
      
      name_set = true;
    }
#ifdef DEBUG
    else {
      Serial.print("Error: Device name not recognized.   ");
      Serial.println(device_name);
      exit(0);
    }
#endif
  }
}