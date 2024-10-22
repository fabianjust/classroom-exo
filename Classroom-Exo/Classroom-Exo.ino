#include "config.h"
#include "messaging.h"
#include "servo_control.h"
#include "sensor_control.h"
#include "battery_management.h"

sensorData mySensors;

// DeviceSettings currentSettings;

void setup() {
  initializeCommunication();
  initializeHardware();
  initBatteryVoltage(&myBat);
  initializeLED();
  getDeviceName();
  initializeSensors(&mySensors);
}

void loop() {
  blinkLED_Button();
  pollBatteryVoltage(&myBat);
  currentSendingTime = millis();
  checkDeviceIntegrity();
  
  receiveMessage();

  if (newData) {
    receiveBigBuffer(&msg2receive);
  }

 
  if (newMessage) {
    if(checkReceivedCommand(&msg2receive, &msg2send)) {
      changeModes(&msg2receive, &msg2send);
    }
  }

  processPIDTrajectory();



  processServoResetting();

  // pi_handler();
  

  handleIMUStream(imu_stream);

  handleSensorStream(sensor_stream, control_mode);

}
