/**
 * @file servo_control.cpp
 * @brief Servo motor control functions.
 * 
 * This file contains functions for initializing and controlling the elbow servo motor.
 */

#include "servo_control.h"

Servo myservo;  // Create servo object to control a servo

PID myPID(&servo_controller.pid_input, &servo_controller.pid_output, &servo_controller.setpoint_target, servo_controller.Kp, servo_controller.Ki, servo_controller.Kd, DIRECT);

boolean servo_state = false;

uint16_t ref_degree = 1800 - 100;;

uint16_t angleStep = 10;

const uint16_t minAngle = 1800;
const uint16_t maxAngle = 1800 - 1100;

uint16_t zeroServoAngle = 1800;

uint8_t zero_counter = 0;

uint8_t control_mode = 0x00;

bool zero_servo = false;

unsigned long currentServoTime = 0;
unsigned long previousServoTime = 0;
unsigned long servo_move_interval = 20; // Update interval in milliseconds

unsigned long lastFilterTime = 0;


unsigned long servo_send_interval = 20;

uint16_t bigBuffer[BIG_BUFFER_SIZE];
uint16_t bigBufferIndex = 0;

uint16_t pid_package_nums = 0;
uint16_t tempbufferval = 0; 
uint16_t pid_indx_counter = 0;
uint16_t pid_package_counter = 0;
uint16_t servoStepCounter = 0;
int16_t step = 0;
uint8_t zeroCounter = 0;
uint16_t PID_steps = 0;
uint16_t pid_curve[800];
uint16_t positionDesired = 1800; 
float previousError = 0;
int lastInput;

uint8_t cycle_counter = 0;

// Step response variables
bool stepResponseActive = false;
unsigned long stepStartTime = 0;
unsigned long withinToleranceTime = 0;  // Time when position first came within tolerance

// New variable to control program start
bool programStarted = false;

// Low-pass filter variables
float filteredPosition; // Initial filtered position (center of 10-bit ADC range)

PIDController_t servo_controller = {
    .pwm_target = 1500,    // Initial target PWM (center position)
    .pwm_current = 1500,   // Initial current PWM
    .pwm_calculated = 0,
    .current_angle = 0.0,
    .setpoint_target = 0.0,
    .pid_output = 0.0,
    .pid_input = 0.0,
    .velocity_current = 0, // Initial velocity
    .integral_error = 0,   // Initial integral error
    .Kp = 0.0,             // Proportional gain (adjust as needed)
    .Ki = 0.0,            // Integral gain (adjust as needed)
    .Kd = 0.0,
    .velocity_limit = 50000, // PWM units/s, adjust based on desired max speed
    .integral_limit = 20000, // Limit for integral term to prevent windup
    .deadband = 5,         // Deadband range (in PWM units)
    .min_pwm = 1050,       // Minimum PWM value (adjust based on your servo)
    .max_pwm = 2300,       // Maximum PWM value (adjust based on your servo)
    .min_step = 8          // Minimum step size (twice the deadband)
};

StepResponse_t step_response = {
    .active = false,
    .startTime = 0,
    .maxDuration = 3000,  // 1 seconds maximum for each step
    .positionTolerance = 5,  // Tolerance for considering position reached (in analog units)
    .settleDuration = 100,  // Time to stay within tolerance (in milliseconds)
    .withinToleranceTime = 0,
    .initialStep = 90.0,  // Initial step to 1750 microseconds
    .finalStep = 0.0,     // Final step to 1250 microseconds
    .currentIteration = 0,
    .totalIterations = 5,  // Total number of iterations (each iteration includes both steps)
    .isInitialStep = true,
    .isWaiting = true,
    .waitStartTime = 0,
    .waitDuration = 1000, // 1 second in milliseconds
};


/**
 * @brief Initialize the servo motor.
 * 
 * This function sets up the servo motor, including attaching it to the correct pin
 * and setting initial position if necessary.
 * 
 * @return true if initialization was successful, false otherwise.
 */
bool initializeServo() {
  myservo.attach(SERVO_CONTROL_PIN);  // Attaches the servo on pin 5 to the servo object
  return true;
}

/**
 * @brief Initializes the servo motor and sets its initial position.
 * 
 * This function performs the following steps:
 * 1. Powers on the servo
 * 2. Reads the current position
 * 3. Converts the position to degrees and microseconds
 * 4. Sets the servo to the correct initial position
 * 5. Attaches the servo to the control pin
 * 
 * @return The current state of the servo after activation (true)
 */
bool activateServo() {
  digitalWrite(SERVO_POWER_PIN, HIGH);                                   
  uint16_t positionIs = analogRead(SERVO_POSITION_PIN);                  
  uint16_t positionIsDeg = adcToDegrees(positionIs, currentSettings); 
  uint16_t current_deg2micros = degreesToMicroseconds(positionIsDeg, currentSettings);
  uint16_t current_servo_micros = myservo.readMicroseconds();
  
  if (current_servo_micros < maxAngleServoMicros || current_servo_micros > minAngleServoMicros) {
        // If position > 180, set to min (2400)
        if (positionIsDeg > 1800) {
            current_servo_micros = minAngleServoMicros; 
        }
        // If position < 70, set to max (1265)
        if (positionIsDeg < 700) {
            current_servo_micros = maxAngleServoMicros;
        }
        else
          current_servo_micros = 2000;
  }
  moveToPositionMicroseconds((isValidPosition(current_deg2micros, currentSettings) ? current_deg2micros : current_servo_micros), currentSettings);

  delay(10);
  #ifdef SERVOMOVEMENT
  myservo.attach(SERVO_CONTROL_PIN, currentSettings.servo110ReferenceMicros, currentSettings.servoZeroReferenceMicros); // Attach the servo to pin 5 & move to approx current position
  #endif

#ifdef DEBUG
  Serial.print("positionIs ");
  Serial.print(positionIs);
  Serial.print("currentPos ");
  Serial.print(current_deg2micros);
  Serial.print("currentPos (read) ");
  Serial.print(current_servo_micros);
  Serial.print("   currentPosDeg ");
  Serial.print(positionIsDeg);
#endif
  positionDesired = positionIsDeg;

  return true;
}

/**
 * @brief Deactivate the servo motor.
 * 
 * This function powers off the servo motor to conserve energy when not in use.
 * 
 * @return The current state of the servo after deactivation (false)
 */
bool deactivateServo() {
  uint16_t current_servo_position = analogRead(SERVO_POSITION_PIN);
  uint16_t current_servo_micros = myservo.readMicroseconds();
  digitalWrite(SERVO_POWER_PIN, LOW); // set servo power pin low
  #ifdef SERVOMOVEMENT
  myservo.detach();                 // detach the servo (now free movement is possible)
  #endif

  return false;
}

/**
 * @brief Move the servo to a specific position.
 * 
 * @param position The desired position in degrees (0-180).
 * @return true if the movement was successful, false otherwise.
 */
bool moveServo(int position) {
  if (position < 0 || position > 180) {
    return false;  // Invalid position
  }
  #ifdef SERVOMOVEMENT
  myservo.write(position);
  #endif
  return true;
}

float adcFloatToDegrees(uint16_t voltread, const DeviceSettings &defaultsettings)
{
  return (float)(180.0 - map(voltread, defaultsettings.servo110ReferenceBytes, defaultsettings.servoZeroReferenceBytes, 70.0, 0.0));
}

// uint16_t adcToDegrees(uint16_t voltread, const DeviceSettings &defaultsettings)
// {
//   return (uint16_t)(1800-map(voltread, defaultsettings.servo110ReferenceBytes, defaultsettings.servoZeroReferenceBytes, 700, 0));
// }

// Find angle using linear interpolation between points from device settings
uint16_t adcToDegrees(uint16_t voltread, const DeviceSettings &defaultsettings) {
    // Handle values outside range
    if (voltread >= defaultsettings.servoCalibration[0].potValue) 
        return defaultsettings.servoCalibration[0].angleX10;
    if (voltread <= defaultsettings.servoCalibration[NUM_CALIBRATION_POINTS-1].potValue) 
        return defaultsettings.servoCalibration[NUM_CALIBRATION_POINTS-1].angleX10;
    
    // Find bracketing points
    for (int i = 0; i < NUM_CALIBRATION_POINTS-1; i++) {
        if (voltread >= defaultsettings.servoCalibration[i+1].potValue) {
            // Linear interpolation
            return (uint16_t)(map(voltread,
                      defaultsettings.servoCalibration[i].potValue,
                      defaultsettings.servoCalibration[i+1].potValue,
                      defaultsettings.servoCalibration[i].angleX10,
                      defaultsettings.servoCalibration[i+1].angleX10));
        }
    }
    // If we somehow get here (should be impossible with valid data)
    Serial.println("ERROR: Failed to map pot value!");
    Serial.print("Pot value: ");
    Serial.println(voltread);
    return defaultsettings.servoCalibration[0].angleX10;  // Return a safe default
}


uint16_t degreesToMicroseconds(uint16_t target_degrees, const DeviceSettings &defaultsettings)
{
  // Map the input angle (degrees) to the pulse width range (adjust as needed)
  uint16_t micros = map(target_degrees, defaultsettings.servo110ReferenceDegrees, defaultsettings.servoZeroReferenceDegrees, defaultsettings.servo110ReferenceMicros, defaultsettings.servoZeroReferenceMicros);
  return (uint16_t)constrain(micros, maxAngleServoMicros+1, minAngleServoMicros-1);
}

uint16_t bytesToMicroseconds(uint16_t target_bytes, const DeviceSettings &defaultsettings)
{
  // Map the input angle (degrees) to the pulse width range (adjust as needed)
  uint16_t temp = (map(target_bytes, defaultsettings.servo110ReferenceBytes, defaultsettings.servoZeroReferenceBytes, defaultsettings.servo110ReferenceMicros, defaultsettings.servoZeroReferenceMicros));
  return (uint16_t)constrain(temp, maxAngleServoMicros+1, minAngleServoMicros-1);
  }

void moveToPositionDeg2Microseconds(uint16_t target, const DeviceSettings& settings)
{
  uint16_t microseconds = degreesToMicroseconds(target, settings);
  Serial.print("micros: ");
  Serial.print(microseconds);
  Serial.print(" ");
  #ifdef DEBUG
  Serial.print("micros: ");
  Serial.print(microseconds);
  #endif


  if(isValidPosition(microseconds, settings)) {
    #ifdef SERVOMOVEMENT
    // Move the servo to the target position in microseconds
    myservo.writeMicroseconds(microseconds);
    #endif
  }
  else{
    microseconds = constrain(microseconds, maxAngleServoMicros+1, minAngleServoMicros-1);
    #ifdef DEBUG
    Serial.print("  new micros: ");
    Serial.println(microseconds);
    #endif
    #ifdef SERVOMOVEMENT
    // Move the servo to the target position in microseconds
    myservo.writeMicroseconds(microseconds);
    #endif
  }
}

void moveToPositionMicroseconds(uint16_t target, const DeviceSettings& settings)
{
  #ifdef DEBUG
  Serial.print("micros: ");
  Serial.print(target);
  #endif
  // Move the servo to the target position in microseconds
  target = constrain(target, settings.servo110ReferenceMicros+1, settings.servoZeroReferenceMicros-1);
  #ifdef DEBUG
  Serial.print("  new micros: ");
  Serial.println(target);
  #endif
  #ifdef SERVOMOVEMENT
  myservo.writeMicroseconds(target);
  #endif
}

/**
 * @brief Checks if the given position is within the valid range for the servo.
 * 
 * @param position The position to check, in microseconds.
 * @param settings The servo settings containing the valid range limits.
 * @return true if the position is valid, false otherwise.
 */
bool isValidPosition(uint16_t position, const DeviceSettings& settings) {
  return position < minAngleServoMicros && position > maxAngleServoMicros;
}

void processPIDTrajectory() {
  if (!pid_stream) {
      return;
  }
  
  unsigned long currentServoTime = millis();
  if (currentServoTime - previousServoTime < servo_move_interval) {
      return;
  }

  previousServoTime = currentServoTime;
  if (pid_curve[pid_indx_counter] < maxAngle && pid_indx_counter < ((PID_steps / 2) - 1)) {
      pid_curve[pid_indx_counter] = 1700;
  }

  if ((servoStepCounter % speed_setting == 0) && (pid_indx_counter < ((PID_steps / 2) - 1))) {
      int16_t step = (pid_curve[pid_indx_counter + 1] - pid_curve[pid_indx_counter]) / speed_setting;
      servoStepCounter = 0;
      pid_indx_counter++;

  } else if (pid_indx_counter >= ((PID_steps / 2) - 1)) {
      moveToPositionDeg2Microseconds(pid_curve[pid_indx_counter], currentSettings);
      pid_stream = false;
      PID_steps = 0;
      pid_indx_counter = 0;
      pollBatteryVoltage(&myBat);
      Serial.println("pid finished");
      return;
  }
  uint16_t positionIs = readServoAngle();
  uint16_t currentPosition = adcToDegrees(positionIs, currentSettings);
  uint16_t desiredPosition = (pid_curve[pid_indx_counter - 1] + (step * servoStepCounter));

  // Calculate error
    float error = desiredPosition - currentPosition;

  moveToPositionDeg2Microseconds(desiredPosition, currentSettings);
  Serial.print(desiredPosition);
  Serial.print(" pid pos moved  ");

  // Apply output to servo
  uint16_t newPosition = currentPosition + (uint16_t)error;
  moveToPositionDeg2Microseconds(newPosition, currentSettings);

  // Update previous error for next iteration
  previousError = error;

  positionIs = readServoAngle();
  currentPosition = adcToDegrees(positionIs, currentSettings);
  
  Serial.print(currentPosition);
  Serial.print(" ");

  if ((servoStepCounter % speed_setting) == 0) {
    #ifdef DEBUG2
    // currentPosition = position;
    #endif
    packageServoData(&msg2send, currentPosition);
    
    sendMessage(&msg2send);
    Serial.println(" pid pos sent ");
  }

  servoStepCounter++;

  return;
}

void pi_handler(){
   if (!PI_programStarted) {
        return;  // Don't proceed if the program hasn't started
    }

    unsigned long currentTime = millis();

    // High-frequency filtering
    if (currentTime - lastFilterTime >= filterInterval) {
        updateFilteredPosition();
        lastFilterTime = currentTime;
    }
   
    if (currentTime - previousServoTime >= servo_move_interval) {
        float dt = (currentTime - previousServoTime) / 1000.0;  // Convert to seconds
       
        // Only update controller if signal generator is active
        if (SignalGenerator_isActive(&signalGen)) {
            // Update signal generator
            SignalGenerator_update(&signalGen, &servo_controller);
            
            // Only compute PID if not in waiting state
            // if (!(signalGen.isWaiting)) {
                servo_controller.pid_input = mapAnalogToDeg(currentSettings);
                myPID.Compute();
                // Set servo position
                // myservo.write(180 - constrain(int(servo_controller.pid_output), 0, 110));
                uint16_t temp_pid_output = 1800 - round(servo_controller.pid_output*10);
                moveToPositionDeg2Microseconds(temp_pid_output, currentSettings);
                degreesToMicroseconds(temp_pid_output, currentSettings);
                int temp = myservo.readMicroseconds();
                Serial.print("meas micros: ");
                Serial.print(temp);
                Serial.print(" ");
            // }
         
            // Print data for plotting
            SignalGenerator_printStatus(&signalGen, &servo_controller);
        
            previousServoTime = currentTime;
          
        }
    }

    // Handle periodic data sending
    if (currentTime - previousSendingTime >= servo_send_interval) {
        if (SignalGenerator_isActive(&signalGen)) {
            cycle_counter++;
            uint16_t positionIs = readServoAngle();
            uint16_t currentPosition = adcToDegrees(positionIs, currentSettings);
           
            // package Servo data
            packageServoDataFloat(&msg2send, servo_controller.pid_input);
            // packageServoDataFloat(&msg2send, servo_controller.setpoint_target);
            sendMessage(&msg2send);
            Serial.print(cycle_counter);
            Serial.print(",");
            Serial.print(filteredPosition);
            Serial.println(" pid pos sent ");
            previousSendingTime = currentTime;
        }
    }

}

void PIDController_tick(PIDController_t *controller, float dt) {
    // Read and filter the actual servo position
    controller->pwm_current = mapAnalogToPWM(currentSettings)-controller->min_pwm;
    Serial.print("pwm current: ");
    Serial.print(controller->pwm_current);

    // Ensure target is within bounds
    controller->pwm_target = constrain(controller->pwm_target, controller->min_pwm, controller->max_pwm);
    Serial.print(" pwm target: ");
    Serial.print(controller->pwm_target);

    // Calculate PWM error based on target PWM and current position
    int pwm_error = controller->pwm_current -(controller->pwm_target-controller->min_pwm);

    // Apply deadband and ensure minimum step size
    if (abs(pwm_error) <= controller->deadband) {
        pwm_error = 0;
    } else {
        // Ensure the error is at least the minimum step size
        int sign = (pwm_error > 0) ? 1 : -1;
        pwm_error = sign * max(abs(pwm_error), controller->min_step);
    }

    Serial.print("  proportional term: ");
    Serial.print(pwm_error);

    // Dynamic Kp (simple gain scheduling)
    float dynamicKp = controller->Kp;
    // if (abs(pwm_error) > 200) {
    //   dynamicKp = controller->Kp * 1.5; // Increase proportional gain for large errors
    // }

    // Update integral error (only if outside deadband)
    if (pwm_error != 0) {
      
      // controller->integral_error += pwm_error * dt;
      // controller->integral_error = CLAMP(controller->integral_error, -controller->integral_limit, controller->integral_limit);
    
      // Anti-windup: Limit integral term
      controller->integral_error += controller->Ki * (pwm_error * dt);
      Serial.print("dt: ");
      Serial.print(dt);
      controller->integral_error = constrain(controller->integral_error, -500, 500);
      Serial.print("  integral term: ");
      Serial.print(controller->integral_error);
    }

    /* Difference of Error - Derivative term */
    // delta_pwm_error = pwm_error - previous_pwm_error;
    int dInput = controller->pwm_current - lastInput; 
   

    // Calculate PI output
    /*Remember some variables for next time*/
    lastInput = controller->pwm_current;

    controller->pwm_calculated = (dynamicKp * pwm_error + controller->integral_error)+controller->min_pwm;
   
    Serial.print("  new PWM ");
    Serial.print(controller->pwm_calculated);

    // // Update velocity based on PI output
    // controller->velocity_current = CLAMP(controller->pwm_calculated, -controller->velocity_limit, controller->velocity_limit);
    // Serial.print(" velocity: ");
    // Serial.print(controller->velocity_current);

    // // Update PWM
    // // int new_pwm = controller->pwm_current + round(controller->velocity_current * dt);
    // int new_pwm = controller->velocity_current;
    
    controller->pwm_calculated = constrain(controller->pwm_calculated, controller->min_pwm, controller->max_pwm);
    Serial.print(" new pwm: ");
    Serial.println(controller->pwm_calculated);
}


void performStepResponse() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - step_response.startTime;

    if (step_response.isWaiting) {
        if (currentTime - step_response.waitStartTime >= step_response.waitDuration) {
            // Waiting period is over, move to the next step
            step_response.isWaiting = false;
            // Reset integral error when moving to next step to prevent accumulated error
            servo_controller.integral_error = 0;
            moveToNextStep();
            Serial.println("  --- waiting done");
        }
        Serial.print("waiting ");
        return; // Don't do anything else while waiting
        
    }
   
    if (isPositionReached() || elapsedTime >= step_response.maxDuration) {
      // Reset integral error before moving to next step
      servo_controller.integral_error = 0;
      // Directly move to next step without waiting
      moveToNextStep();
      return;
      //  // Position reached or max duration exceeded, start waiting period
      //   // step_response.isWaiting = true;
      //   step_response.waitStartTime = currentTime;

      //   if (step_response.isWaiting) {
      //     if (currentTime - step_response.waitStartTime >= step_response.waitDuration) {
      //       // Waiting period is over, move to the next step
      //       step_response.isWaiting = false;
      //       moveToNextStep();
      //     }
      //   return; // Don't do anything else while waiting
      //  }

    }
}


void moveToNextStep() {
    if ((step_response.isInitialStep) && (step_response.currentIteration < step_response.totalIterations)) {
        // Switch to final step
        servo_controller.setpoint_target = step_response.initialStep;
        step_response.isInitialStep = false;
        Serial.println("#############################initial step##########################");
    } else {
        
        
        if (step_response.currentIteration < step_response.totalIterations) {
            // Start next iteration
            servo_controller.setpoint_target = step_response.finalStep;
            step_response.isInitialStep = true;
            Serial.println("######################final step################");
            // Completed one full iteration
            step_response.currentIteration++;
        } else {
            // End the step response test
            endStepResponse();
        }
    }
    
    step_response.startTime = millis();
    step_response.withinToleranceTime = 0;
}

void endStepResponse() {
    step_response.active = false;
    PI_programStarted = false;
    // send current oposition one last time
    uint16_t positionIs = readServoAngle();
    uint16_t currentPosition = adcToDegrees(positionIs, currentSettings);
    // package Servo data
    packageServoDataFloat(&msg2send, servo_controller.pid_input);
    sendMessage(&msg2send);
    Serial.println(" pid pos sent ");

    Serial.println("Step response test completed.");
    pollBatteryVoltage(&myBat);
    
    servo_controller = {
                    .pwm_target = 1500,    // Initial target PWM (center position)
                    .pwm_current = 1500,   // Initial current PWM
                    .pwm_calculated = 0,
                    .current_angle = 0.0,
                    .setpoint_target = 0.0,
                    .pid_output = 0.0,
                    .pid_input = 0.0,
                    .velocity_current = 0, // Initial velocity
                    .integral_error = 0,   // Initial integral error
                    .Kp = 1.1,             // Proportional gain (adjust as needed)
                    .Ki = 0.5, 
                    .Kd = 0.0,           // Integral gain (adjust as needed)
                    .velocity_limit = 50000, // PWM units/s, adjust based on desired max speed
                    .integral_limit = 20000, // Limit for integral term to prevent windup
                    .deadband = 5,         // Deadband range (in PWM units)
                    .min_pwm = 1050,       // Minimum PWM value (adjust based on your servo)
                    .max_pwm = 2300,       // Maximum PWM value (adjust based on your servo)
                    .min_step = 8          // Minimum step size (twice the deadband)
                };
    
    // send stop bit
    msg2send.header = HEADER;
    msg2send.opcode = RESP_PID_FINISHED;
    msg2send.dataLength = 0;
    sendMessage(&msg2send);
}

bool isPositionReached() {
    int error = abs(servo_controller.pwm_target - servo_controller.pwm_calculated);
    
    if (error <= step_response.positionTolerance) {
        if (step_response.withinToleranceTime == 0) {
            step_response.withinToleranceTime = millis();
        }
        return (millis() - step_response.withinToleranceTime >= step_response.settleDuration);
    } else {
        step_response.withinToleranceTime = 0;
        return false;
    }
}


void updateFilteredPosition() {
    float rawPosition = analogRead(SERVO_POSITION_PIN);
    filteredPosition = alpha * rawPosition + (1 - alpha) * filteredPosition;
}

int mapAnalogToPWM(const DeviceSettings &defaultsettings) {
  return map(round(filteredPosition), defaultsettings.servoZeroReferenceBytes, defaultsettings.servo110ReferenceBytes, defaultsettings.servoZeroReferenceMicros, defaultsettings.servo110ReferenceMicros);
  // return myservo.readMicroseconds();
}

float mapAnalogToDeg(const DeviceSettings &defaultsettings) {
  uint16_t temp = (1800 - map(round(filteredPosition), defaultsettings.servoZeroReferenceBytes, defaultsettings.servo110ReferenceBytes, defaultsettings.servoZeroReferenceDegrees, defaultsettings.servo110ReferenceDegrees));
  Serial.print("filted Pos:");
  Serial.print(filteredPosition);
  Serial.print("temp Pos:");
  Serial.print(temp);
  return (float)temp/10;
}


void startStepResponse() {
    step_response.isWaiting = true;
    step_response.waitStartTime = millis();
    step_response.active = true;
    step_response.startTime = millis();
    step_response.withinToleranceTime = 0;
    step_response.currentIteration = 0;
    step_response.isInitialStep = true;
    servo_controller.pwm_target = step_response.initialStep;
    filteredPosition = readServoAngle();
    previousServoTime = millis();
    previousSendingTime = millis();
    cycle_counter = 0;
}

void packageServoData(message *msgOut, uint16_t currentPosition) {
  msgOut->opcode = SERVO_ANSWER;
  msgOut->dataLength = 2;

  memcpy(&msgOut->data[0], &currentPosition, sizeof(uint16_t));
  
}

void packageServoDataFloat(message *msgOut, float currentPosition) {
  msgOut->opcode = SERVO_ANSWER;
  msgOut->dataLength = 4;

  memcpy(&msgOut->data[0], &currentPosition, sizeof(float));
  
}


void processServoResetting() {
  if (!zero_servo) {
        return;  // Zeroing is already complete
    }

  unsigned long currentServoTime = millis();
    if (currentServoTime - previousServoTime < servo_move_interval) {
        return;
    }

  uint16_t currentPosition = adcToDegrees(readServoAngle(), currentSettings); 
  uint16_t microseconds = degreesToMicroseconds(currentPosition, currentSettings); 
  positionDesired = currentPosition + angleStep;
  microseconds = bytesToMicroseconds(readServoAngle(), currentSettings); 

  #ifdef DEBUG
  Serial.print(positionDesired);
  Serial.print("   ");
  Serial.print(currentPosition);
  Serial.print("   ");
  Serial.print(microseconds);
  Serial.print("   ");
  Serial.println(myservo.readMicroseconds()); 
  #endif

  if ((currentPosition < zeroServoAngle) || (positionDesired < zeroServoAngle)) {
      positionDesired = zeroServoAngle;
      pollBatteryVoltage(&myBat);
      #ifdef DEBUG
      Serial.println("Servo zeroed!");
      #endif
      zero_servo = false;
  }

  #ifdef DEBUG
  Serial.print("moved servo with elapsed time:  ");
  Serial.println(currentServoTime - previousServoTime);
  #endif

  previousServoTime = currentServoTime;

  
  moveToPositionDeg2Microseconds(positionDesired, currentSettings);
  

  zeroCounter++;

  #ifdef DEBUG2
  if (zeroCounter >= 25) {
      zero_servo = false;
      Serial.println("Servo zeroed!");
  }
  #endif
}


void updateServoPosition(uint8_t control_mode, sensorData *data) {
  switch (control_mode)
  {
  case CMD_BI_EMG_STREAM:
    biThresholdBicontrol(data->emgAvg[0],data->emgAvg[1], emgThresholdUpper, emgThresholdLower);
    break;

  case CMD_FORCE_STREAM:
    biThresholdControl(data->forceAvg, forceThresholdUpper, forceThresholdLower);
    break;
  
  case CMD_MONO_EMG_STREAM:
    biThresholdControl(data->emgAvg[0], emgThresholdUpper, emgThresholdLower);
    break;
  
  default:
    // biThresholdControl();
    break;
  }
}

// Add these initialization functions to your setup or where you start the PI control
void initializeSignalGenerator(SignalType type) {
  Serial.print("initialize signal gen: ");
  Serial.println(type);

    // Initialize signal generator for step response
    SignalGenerator_init(&signalGen, type);
    
    // Configure step response parameters
    SignalGenerator_setStepValues(&signalGen, 0.0, 90.0);  // Example: 0 to 90 degrees
    SignalGenerator_setWaitTime(&signalGen, 2000);         // 2 second wait time
    SignalGenerator_setMaxDuration(&signalGen, 5000);      // 5 second max duration per step
    SignalGenerator_setIterations(&signalGen, 3);          // 3 complete cycles
}
  
