#include "messaging.h"


boolean MatlabConnected = false;
boolean EmergencyStop = false;
boolean newMessage = false;
boolean newData = false;
const byte startMarker = 0x3C;
const byte endMarker = 0x3E;
uint8_t byteIndex = 0;
boolean recvInProgress = false;
const byte numBytes = 32;
byte receivedBytes[numBytes];

uint8_t packageLength = 4; // default package length
uint8_t numReceived = 0;

unsigned long currentSendingTime = 0;
unsigned long previousSendingTime = 0;
unsigned long sending_interval = 50;

bool zero_force = false;
bool pid_stream = false;
bool PI_programStarted = false;
bool emg_stream = false;
bool force_stream = false;
bool bi_emg_stream_min_max = false;
bool bi_emg_stream_increment = false;
bool emg_stream_increment = false;
bool emg_stream_min_max = false;
bool imu_stream = false;
bool sensor_stream = false;



void initializeCommunication() {
  pinMode(NINA_RESETN, OUTPUT);
  digitalWrite(NINA_RESETN, HIGH);
  // Initialize communication protocols
  Serial.begin(BAUD_RATE);
  SerialNina.begin(BAUD_RATE);
while (!Serial && millis() < 100)
  ;
Serial.println("Serial connected");
}

void getDeviceName() {
  while (device_name == "") {
    if (SerialNina.available()) {
      String input = SerialNina.readStringUntil('\n');
      input.trim();
      if (input.startsWith("DEVICE_NAME:")) {
        device_name = input.substring(strlen("DEVICE_NAME:"));
        Serial.println(device_name);
        dev_ok = true;
      } else {
        Serial.println("Error: Invalid input received. Waiting for correct device name format.");
        Serial.println(input);
      }
    }
  }
}

void receiveMessage() {
  byte rb;
  // while ((SerialNina.available() > 0) && newData == false)
  if ((SerialNina.available() > 0) && newData == false)
  {
    rb = SerialNina.read();
#ifdef DEBUG
    Serial.print(rb, HEX);
    Serial.println(byteIndex);
#endif
    

    if (recvInProgress)
    {
      if ((byteIndex <= packageLength - 1))
      {
        // Store the byte in the big buffer
        // storeByteInBigBuffer(rb);
        receivedBytes[byteIndex] = rb;
        if (byteIndex == 2)
        {
          packageLength += rb;
        }
#ifdef DEBUG
        Serial.print("Index: ");
        Serial.print(byteIndex);
        Serial.print("  Byte: ");
        Serial.println(receivedBytes[byteIndex]);
        Serial.print("Packagelength: ");
        Serial.println(packageLength);
#endif
        byteIndex++;
        // bigBufferIndex++;
        if (byteIndex >= numBytes)
        {
          byteIndex = numBytes - 1;
        }
      }

      // else if ((rb == endMarker) || (byteIndex >= packageLength-1)) {
      else if ((byteIndex >= packageLength-1)) 
      {
        receivedBytes[byteIndex] = '\0'; // terminate the string
        // storeByteInBigBuffer('\0');
        recvInProgress = false;
        packageLength = 4; // default package length
        numReceived = byteIndex; // save the number for use when printing
        byteIndex = 0;
        // bigBufferIndex=0;
        newData = true;
        MatlabConnected = true;
        
#ifdef DEBUG
        Serial.println("recv complete...");
        Serial.println(bigBufferIndex);
#endif
      }
    }

    else if (rb == startMarker)
    {
      recvInProgress = true;
      byteIndex = 0;
    }
  }
}


void receiveBigBuffer(message *msgIn) {
  if (newData) {
#ifdef DEBUG
    Serial.print("This just in (HEX values)... ");
    for (byte n = 0; n < numReceived; n++)
    {
      Serial.print(receivedBytes[n], HEX);
      Serial.print(' ');
    }
    Serial.println();
#endif
    // parseData();
    newData = false;
    newMessage = false;
    // numReceived = 0;
    byte ndex = 0;

    msgIn->header = receivedBytes[ndex];
    msgIn->opcode = receivedBytes[ndex + 1];
#ifdef DEBUG
    Serial.println(msgIn->header);
    Serial.println(msgIn->opcode);
#endif

    if (msgIn->opcode == 0x12)
    {
      pid_package_counter++;
#ifdef DEBUG
      Serial.print(numReceived - 4);
      Serial.println("Saving to Big Buffer");
#endif
      msgIn->dataLength = receivedBytes[ndex + 2];
      PID_steps += msgIn->dataLength;
      for (int i = 0; i < msgIn->dataLength; i++)
      {
        storeByteInBigBuffer(receivedBytes[i + 3u + ndex]);
#ifdef DEBUG
        Serial.print(receivedBytes[i + 3u + ndex]);
        Serial.print(" ");
#endif
      }
#ifdef DEBUG
      Serial.println();
      Serial.println(msgIn->dataLength);
#endif

      msgIn->checksum = receivedBytes[(numReceived - 4) + 3 + ndex];
      newMessage = true;
    }

    else if (!(msgIn->opcode == 0x01))
    {
      msgIn->dataLength = receivedBytes[ndex + 2];

      // msgIn->data[msgIn->dataLength];

      for (int i = 0; i < msgIn->dataLength; i++)
      {
        msgIn->data[i] = receivedBytes[i + 3u + ndex];
        // Serial.println(msgIn->data[i]);
#ifdef DEBUG
        Serial.print("data:");
        Serial.println(receivedBytes[i + 3u + ndex]);
#endif
      }

#ifdef DEBUG
      Serial.print("data len:");
      Serial.println(msgIn->dataLength);
#endif

      msgIn->checksum = receivedBytes[msgIn->dataLength + 3u + ndex];
      newMessage = true;
    }

    else
    {
      // msgIn->opcode = 0x01;
      msgIn->dataLength = 0;
      msgIn->checksum = 0;
      newMessage = true;
    }

    // calculatedChecksum = calculateChecksum(&msgOut);
  }
}

void sendMessage(message *msgOut) {
  byte checksum = 0;

  checksum = msgOut->header + msgOut->opcode + msgOut->dataLength;
  msgOut->checksum = checksum;
  // calculateChecksum(header, opcode, length);

  uint8_t totalLength = 1 + 1 + 1 + 1 + msgOut->dataLength + 1 + 1;
  byte message[totalLength];

  uint8_t index = 0;

  // Fill the byte array with the message components
  message[index++] = startMarker;
  message[index++] = msgOut->header;
  message[index++] = msgOut->opcode;
  message[index++] = msgOut->dataLength;

  for (uint8_t i = 0; i < msgOut->dataLength; i++)
  {
    message[index++] = msgOut->data[i];
  }

  message[index++] = msgOut->checksum;
  message[index++] = endMarker;

  // Send the entire message with one Serial.write command
  SerialNina.write(message, totalLength);

#ifdef DEBUG
  Serial.println("sent to Matlab ...");
  Serial.print("HEADER:  ");
  Serial.println(msgOut->header);
  Serial.print("OPCODE:  ");
  Serial.println(msgOut->opcode);
  Serial.print("Length:  ");
  Serial.println(msgOut->dataLength);
#endif
}

bool checkReceivedCommand(message *msgIn, message *msgOut) {
  uint8_t temp = msgIn->checksum;
  uint8_t newCheckSum = msgIn->header + msgIn->opcode + msgIn->dataLength;

  if (temp == newCheckSum) {
    msgOut->opcode = COMMAND_OK;
    return true;
  }

  else {
    msgOut->opcode = WRONG_CHECKSUM;
    return false;
  }
}

void changeModes(message *msgIn, message *msgOut){
  // execute the command and prepare the answer back
  msgOut->header = HEADER;
  msgOut->dataLength = 0;

  if ((msgIn->header == 0xFF) && (msgOut->opcode == COMMAND_OK)) {
      switch (msgIn->opcode) {
        case TEST_CONNECT: {
#ifdef DEBUG
          Serial.println("0x01...");
#endif
          digitalWrite(LED_PIN, HIGH);
          delay(1);
          digitalWrite(LED_PIN, LOW);
          delay(1);
          digitalWrite(LED_PIN, HIGH);
          delay(1);
          digitalWrite(LED_PIN, LOW);

          msgOut->opcode = VERIF_CONNECTION;
        }
        break;
        
        case STOP_CONNECT: {
  #ifdef DEBUG
          Serial.println("0x15...Stopped connection");
  #endif
          changeLEDButtonColor(0x10);

          msgOut->opcode = VERIF_CONNECTION;
          MatlabConnected = false;
        }
        break;

        case SET_LED: {
          uint8_t color = msgOut->data[0];
          changeLEDButtonColor(color);

          msgOut->opcode = STATE_ANSWER;
          msgOut->data[0] = LED_SET;
          msgOut->dataLength = 1;
        }
        break;

        case GET_BAT_VOLTAGE: { // read actual Battery Voltage in mV
          memcpy(&msgOut->data[0], &myBat.batAverag, sizeof(float));
          msgOut->dataLength = 4;
        }
        break;

        case EMERGENCY_STOP: { // read actual Battery Voltage in mV
          memcpy(&msgOut->data[0], &myBat.batAverag, sizeof(float));
          msgOut->dataLength = 4;

          // Turn LED off
          changeLEDButtonColor(0x10);

          // turn off servo motor
          servo_state = deactivateServo();

          // set all modes to false
          MatlabConnected = false;
          zero_force = false;
          pid_stream = false;
          bi_emg_stream_min_max = false;
          bi_emg_stream_increment = false;
          emg_stream_increment = false;
          emg_stream_min_max = false;
          EmergencyStop = true;

          Serial.println("0x23...Emergency Stop");
        }
        break;

        case SET_THRESHOLDS: {
          switch (msgIn->data[0]){
            case FORCE_MODE: {
              forceThresholdUpper = (msgIn->data[2] << 8u) + msgIn->data[1];
              forceThresholdLower = ((msgIn->data[4] << 8u) + msgIn->data[3])*-1;
              speed_setting = msgIn->data[5];
              angleStep = msgIn->data[6];
    
    #ifdef DEBUG
              Serial.print("Upper forceThreshold: ");
              Serial.print(forceThresholdUpper);
              Serial.print("     Lower forceThreshold: ");
              Serial.print(forceThresholdLower);
              Serial.print("     Speed: ");
              Serial.print(speed_setting);
              Serial.print("     AngleStep: ");
              Serial.println(angleStep);
    #endif
            }
            break;

            case MONO_EMG_MODE: {
              emgThreshold = (msgIn->data[2] << 8u) + msgIn->data[1];
              speed_setting = msgIn->data[3];
              angleStep = msgIn->data[4];

            }
            break;

            case BI_EMG_MODE: {
              emgThresholdUpper = (msgIn->data[2] << 8u) + msgIn->data[1];
              emgThresholdLower = (msgIn->data[4] << 8u) + msgIn->data[3];
              speed_setting = msgIn->data[5];
              angleStep = msgIn->data[6];
    #ifdef DEBUG
              Serial.print("Upper emgThreshold: ");
              Serial.print(emgThresholdUpper);
              Serial.print("     Lower emgThreshold: ");
              Serial.print(emgThresholdLower);
              Serial.print("     Speed: ");
              Serial.print(speed_setting);
              Serial.print("     AngleStep: ");
              Serial.println(angleStep);
    #endif
            }
            break;

            default: {
              forceThresholdUpper = 30;
              forceThresholdLower = -30;
              emgThresholdUpper = 30;
              emgThresholdLower = 30;
              emgThreshold = 500;
              speed_setting = 1;
              angle_step = 10;
              msgOut->data[0] = 0x00;
            }
            break;
          }

          msgOut->opcode = COMMAND_OK;
          msgOut->data[0] = msgIn->data[0];
          msgOut->dataLength = 1;
          }
        break;

        case SET_ANALOG_PINS: {
          uint8_t mode = msgIn->data[0];
          if (mode == 0x40) {
            uint8_t temp_pin = msgIn->data[1];
            if (temp_pin == 1) {
              EMG0_SELECT_PIN = EMG_SENSOR0_PIN;
              EMG1_SELECT_PIN = EMG_SENSOR1_PIN;
            }
            else if (temp_pin == 2) {
              EMG0_SELECT_PIN = EMG_SENSOR1_PIN;
              EMG1_SELECT_PIN = EMG_SENSOR0_PIN;
            }

  #ifdef DEBUG
            Serial.print("Seletced Pin: ");
            Serial.println(EMG0_SELECT_PIN);
  #endif
          }

          msgOut->opcode = COMMAND_OK;
          msgOut->data[0] = COMMAND_OK;
          msgOut->dataLength = 1;
        }
        break;

        case ACTV_SERVO: {
          uint8_t set_servo_state = msgIn->data[0];
  #ifdef DEBUG
          Serial.print("servo:");
          Serial.println(set_servo_state);
  #endif
          if ((set_servo_state == SERVO_ACTIVATED))
          {
  #ifdef DEBUG
            Serial.println("Servo activated");
  #endif
            servo_state = activateServo();
            msgOut->data[0] = SERVO_ACTIVATED;
            msgOut->dataLength = 1;
          }
          else if ((set_servo_state == SERVO_DEACTIVATED))
          {
  #ifdef DEBUG
            Serial.println("Servo deactivated");
  #endif
            servo_state = deactivateServo();
            msgOut->data[0] = SERVO_DEACTIVATED;
            msgOut->dataLength = 1;
          }
          else if (servo_state)
          {
  #ifdef DEBUG
            Serial.println("Servo deactivated 2");
  #endif
            servo_state = deactivateServo();
            msgOut->data[0] = SERVO_DEACTIVATED;
            msgOut->dataLength = 1;
          }
          msgOut->opcode = COMMAND_OK;
        }
        break;

        case ZERO_SERVO: {
          ref_degree = (msgIn->data[1] << 8u) + msgIn->data[0];
          speed_setting = msgIn->data[2];
          angleStep = msgIn->data[3];

          zeroServoAngle = ref_degree - 100;
          if (zeroServoAngle > minAngle)
          {
            zeroServoAngle = minAngle;
          }
  #ifdef DEBUG
          Serial.print("speed: ");
          Serial.print(speed_setting);
          Serial.print("  angle_step: ");
          Serial.print(angleStep);
          Serial.print("  ref_angle: ");
          Serial.println(zeroServoAngle);
          Serial.println("zero servo activated!");
  #endif

          zero_counter = 0;
          zero_servo = true;
          previousServoTime = millis();

          if (zero_servo)
            changeLEDButtonColor(0x01);
          else if (!zero_servo)
          {
            pollBatteryVoltage(&myBat);
          }

          msgOut->opcode = COMMAND_OK;
          msgOut->data[0] = ZERO_SERVO;
          msgOut->dataLength = 1;
        }
        break;

        case SET_SPEED: {
          speed_setting = msgIn->data[0];
  #ifdef DEBUG
          Serial.print("speed: ");
          Serial.println(speed_setting);
  #endif
          msgOut->data[0] = COMMAND_OK;
          msgOut->dataLength = 1;
        }
        break;

        case SET_PACKAGE_COUNTER: {
          pid_package_nums = msgIn->data[0];
  #ifdef DEBUG
          Serial.print("nums of pid packages: ");
          Serial.println(pid_package_nums);
  #endif
          msgOut->opcode = COMMAND_OK;
          msgOut->data[0] = SET_SPEED;
          msgOut->dataLength = 1;
        }
        break;

        case MOVE_PID: {
          if ((bigBufferIndex >= (PID_steps / 2)) && (pid_package_counter >= pid_package_nums))
          { //(msgOut.dataLength+117)
            pid_stream = true;
            changeLEDButtonColor(0x10);
            uint16_t j = 0;
  #ifdef DEBUG
            Serial.print(tempbufferval);
            Serial.print(" ");
            Serial.println("writing buffer");
  #endif
            for (uint16_t i = 0; i < (bigBufferIndex - 1); i = i + 2)
            {
              pid_curve[j] = (bigBuffer[i + 1] << 8u) + bigBuffer[i];
  #ifdef DEBUG
              Serial.print(j);
              Serial.print(" ");
              Serial.print(pid_curve[j]);
              Serial.println();
  #endif
              j++;
            }
            bigBufferIndex = 0; // reset bigbuffer index
            pid_package_counter = 0;
            pid_indx_counter = 0; // reset pid index for trajectory
            servoStepCounter = 0;
          }
  #ifdef DEBUG
          else
          {
            Serial.print(bigBufferIndex);
            Serial.print("..Not yet BufferIndex: ");
            Serial.println(PID_steps);
          }
  #endif
          msgOut->data[0] = MOVE_PID;
          msgOut->dataLength = 1;
        }
        break;

        case SET_PI_CONTROLLER_PARAMS: {
          int temp = (msgIn->data[1] << 8u) + msgIn->data[0];
          step_response.initialStep = degreesToMicroseconds(temp, currentSettings);
          temp = (msgIn->data[3] << 8u) + msgIn->data[2];
          step_response.finalStep = degreesToMicroseconds(temp, currentSettings);

          step_response.totalIterations = (msgIn->data[5]<< 8u) + msgIn->data[4];

          servo_controller.Kp = bytesToFloatFromArray(msgIn->data, 6);
          servo_controller.Ki = bytesToFloatFromArray(msgIn->data, 10);

          #ifdef DEBUG
          Serial.print(step_response.initialStep);
          Serial.print("  ,");
          Serial.print(step_response.finalStep);
          Serial.print("  ,");
          Serial.print(step_response.totalIterations);
          Serial.print("  ,");
          Serial.print(servo_controller.Kp);
          Serial.print("  ,");
          Serial.print(servo_controller.Ki);
          Serial.print("  ,");
          Serial.print("pi params set");
          #endif
          msgOut->data[0] = MOVE_PID;
          msgOut->dataLength = 1;
        }
        break;

        case START_PICONTROLLER: {
          servo_controller.integral_error = 0;
          servo_controller.pwm_target = step_response.initialStep,    
          servo_controller.pwm_current = step_response.initialStep,   
          servo_controller.velocity_current = 0, 
          servo_controller.integral_error = 0, 
          #ifdef DEBUG
          Serial.print("start pi control");
          #endif
          msgOut->data[0] = MOVE_PID;
          msgOut->dataLength = 1; 

          PI_programStarted = true;
          startStepResponse();
        }
        break;

        case START_AQUISITION: {
          // change state of stream
          switch(msgIn->data[0]) {
            case FORCE_MODE: {
              sensor_stream = true;
              control_mode = FORCE_MODE;
              changeLEDButtonColor(0x09);
              #ifdef DEBUG
              Serial.print("start force control");
              #endif
              msgOut->data[0] = FORCE_STREAM;
              msgOut->dataLength = 1;
            } break;

            case MONO_EMG_MODE: {
              sensor_stream = true;
              control_mode = MONO_EMG_MODE;
              changeLEDButtonColor(0x02);
              #ifdef DEBUG
              Serial.print("start mono-sensor emg_stream_increment control");
              #endif
              msgOut->data[0] = MONO_EMG_STREAM;
              msgOut->dataLength = 1;
            } break;

            case BI_EMG_MODE: {
              sensor_stream = true;
              control_mode = BI_EMG_MODE;
              changeLEDButtonColor(0x02);
              #ifdef DEBUG
              Serial.print("start bi-sensor emg_stream_increment control");
              #endif
              msgOut->data[0] = BI_EMG_STREAM;
              msgOut->dataLength = 1;
            } break;

            case IMU_MODE: {
              sensor_stream = false;
              control_mode = IMU_MODE;
              changeLEDButtonColor(0x10);
              imu_stream = true;
              #ifdef DEBUG
              Serial.print("start imu stream");
              #endif
              msgOut->data[0] = IMU_STREAM;
              msgOut->dataLength = 1;
            } break;

            default:
              sensor_stream = false;
              imu_stream = false;
              control_mode = 0x00;
              changeLEDButtonColor(0x00);  
              msgOut->data[0] = STOP_AQUISITION;
            msgOut->dataLength = 1;        
          }
        }
        break;

        case STOP_AQUISITION: {
          sensor_stream = false;
          imu_stream = false;
          control_mode = 0x00;
          changeLEDButtonColor(0x00); 
          msgOut->data[0] = STOP_AQUISITION;
          msgOut->dataLength = 1;
          Serial.println("stream stopped");
        }
        break;

    }
    sendMessage(msgOut);
    newMessage = false;
  }
}

void storeByteInBigBuffer(byte data_byte) {
  // Check if there is enough space in the big buffer
  if (bigBufferIndex < BIG_BUFFER_SIZE)
  {
    // Store the byte in the big buffer
    bigBuffer[bigBufferIndex] = data_byte;
    bigBufferIndex++;
    if (!(bigBufferIndex % 2))
    {
      tempbufferval = (bigBuffer[bigBufferIndex] << 8u) + bigBuffer[bigBufferIndex - 1]; // (bigBuffer[i + 1] << 8u) + bigBuffer[i];
    }
  }
  else if (bigBufferIndex >= BIG_BUFFER_SIZE)
  {
    bigBufferIndex = BIG_BUFFER_SIZE - 1;
  }
}



float bytesToFloatFromArray(byte* array, int startIndex) {
  float result;
  memcpy(&result, &array[startIndex], 4);
  return result;
}