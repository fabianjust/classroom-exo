/**
 * @file servo_control.cpp
 * @brief Servo motor control functions.
 * 
 * This file contains functions for initializing and controlling the elbow servo motor.
 */

#include "servo_control.h"

Servo myservo;  // Create servo object to control a servo

boolean servo_state = false;

uint16_t ref_degree = 1800 - 100;;

uint16_t angleStep = 10;

const uint16_t minAngle = 1800 - 100;
const uint16_t maxAngle = 1800 - 1200;

uint16_t zeroServoAngle = 1800 - 100;;

uint8_t zero_counter = 0;

uint8_t control_mode = 0x00;

bool zero_servo = false;

unsigned long currentServoTime = 0;
unsigned long previousServoTime = 0;
unsigned long servo_move_interval = 5; // Update interval in milliseconds

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
uint16_t positionDesired = 900; 
float previousError = 0;

uint8_t cycle_counter = 0;

// Step response variables
bool stepResponseActive = false;
unsigned long stepStartTime = 0;
unsigned long withinToleranceTime = 0;  // Time when position first came within tolerance

// New variable to control program start
bool programStarted = false;

// Low-pass filter variables
float filteredPosition = 512; // Initial filtered position (center of 10-bit ADC range)

PIController_t servo_controller = {
    .pwm_target = 1500,    // Initial target PWM (center position)
    .pwm_current = 1500,   // Initial current PWM
    .velocity_current = 0, // Initial velocity
    .integral_error = 0,   // Initial integral error
    .Kp = 50,             // Proportional gain (adjust as needed)
    .Ki = 0.02,            // Integral gain (adjust as needed)
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
    .maxDuration = 15000,  // 15 seconds maximum for each step
    .positionTolerance = 10,  // Tolerance for considering position reached (in analog units)
    .settleDuration = 1000,  // Time to stay within tolerance (in milliseconds)
    .withinToleranceTime = 0,
    .initialStep = 1750,  // Initial step to 1750 microseconds
    .finalStep = 1250,     // Final step to 1250 microseconds
    .currentIteration = 0,
    .totalIterations = 5,  // Total number of iterations (each iteration includes both steps)
    .isInitialStep = true
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
  
  moveToPositionMicroseconds((isValidPosition(current_deg2micros, currentSettings) ? current_deg2micros : current_servo_micros), currentSettings);

#ifdef DEBUG
  Serial.print("currentPos ");
  Serial.print(current_servo_micros);
  Serial.print("   currentPosDeg ");
  Serial.print(positionIsDeg);
#endif
  #ifdef SERVOMOVEMENT
  myservo.attach(SERVO_CONTROL_PIN, currentSettings.servo140ReferenceMicros, currentSettings.servoZeroReferenceMicros); // Attach the servo to pin 5 & move to approx current position
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

uint16_t adcToDegrees(uint16_t voltread, const DeviceSettings &defaultsettings)
{
  return (uint16_t)(1900 - map(voltread, defaultsettings.servo140ReferenceBytes, defaultsettings.servoZeroReferenceBytes, 1000, 100));
}

uint16_t degreesToMicroseconds(uint16_t target_degrees, const DeviceSettings &defaultsettings)
{
  // Map the input angle (degrees) to the pulse width range (adjust as needed)
  uint16_t degrees = map(target_degrees, 500, 1800, defaultsettings.servo140ReferenceMicros, defaultsettings.servoZeroReferenceMicros);
  return (uint16_t)constrain(degrees, maxAngleServoMicros+1, minAngleServoMicros-1);
}

uint16_t bytesToMicroseconds(uint16_t target_bytes, const DeviceSettings &defaultsettings)
{
  // Map the input angle (degrees) to the pulse width range (adjust as needed)
  uint16_t temp = (map(target_bytes, defaultsettings.servo140ReferenceBytes, defaultsettings.servoZeroReferenceBytes, defaultsettings.servo140ReferenceMicros, defaultsettings.servoZeroReferenceMicros));
  return (uint16_t)constrain(temp, maxAngleServoMicros+1, minAngleServoMicros-1);
  }

void moveToPositionDeg2Microseconds(uint16_t target, const DeviceSettings& settings)
{
  uint16_t microseconds = degreesToMicroseconds(target, settings);
  Serial.print("micros: ");
  Serial.print(microseconds);


  if(isValidPosition(microseconds, settings)) {
    #ifdef SERVOMOVEMENT
    // Move the servo to the target position in microseconds
    myservo.writeMicroseconds(microseconds);
    #endif
  }
  else{
    microseconds = constrain(microseconds, maxAngleServoMicros+1, minAngleServoMicros-1);
    Serial.print("  new micros: ");
    Serial.println(microseconds);
    #ifdef SERVOMOVEMENT
    // Move the servo to the target position in microseconds
    myservo.writeMicroseconds(microseconds);
    #endif
  }
}

void moveToPositionMicroseconds(uint16_t target, const DeviceSettings& settings)
{
  Serial.print("micros: ");
  Serial.print(target);
  // Move the servo to the target position in microseconds
  target = constrain(target, settings.servo140ReferenceMicros+1, settings.servoZeroReferenceMicros-1);
  Serial.print("  new micros: ");
  Serial.println(target);
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
    currentPosition = position;
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
   
    if (currentTime - previousServoTime >= servo_move_interval) {
        float dt = (currentTime - previousServoTime) / 1000.0;  // Convert to seconds
       
        // Only update controller if step response is active
        if (step_response.active) {
            performStepResponse();
            PIController_tick(&servo_controller, dt);
        
            // Set servo position
            myservo.writeMicroseconds(servo_controller.pwm_current);
        
            // Print data for plotting
            Serial.print(currentTime);
            Serial.print(",");
            Serial.print(servo_controller.pwm_target);
            Serial.print(",");
            Serial.print(servo_controller.pwm_current);
            Serial.print(",");
            Serial.println(servo_controller.pwm_current);
        
            previousServoTime = currentTime;
        }
    }

}

void PIController_tick(PIController_t *controller, float dt) {
    // Read and filter the actual servo position
    controller->pwm_current= mapAnalogToPWM();

    // Calculate PWM error based on target PWM and current position
    int pwm_error = controller->pwm_target - controller->pwm_current;
    
    // Apply deadband and ensure minimum step size
    if (abs(pwm_error) <= controller->deadband) {
        pwm_error = 0;
    } else {
        // Ensure the error is at least the minimum step size
        int sign = (pwm_error > 0) ? 1 : -1;
        pwm_error = sign * max(abs(pwm_error), controller->min_step);
    }
   
    // Update integral error (only if outside deadband)
    if (pwm_error != 0) {
        controller->integral_error += pwm_error * dt;
        controller->integral_error = constrain(controller->integral_error, -controller->integral_limit, controller->integral_limit);
    }
   
    // Calculate PI output
    float output = controller->Kp * pwm_error + controller->Ki * controller->integral_error;
   
    // Update velocity based on PI output
    controller->velocity_current = constrain(output, -controller->velocity_limit, controller->velocity_limit);
   
    // Update PWM
    int new_pwm = controller->pwm_current + round(controller->velocity_current * dt);
    
    // Ensure the PWM change is at least the minimum step size if there's any change at all
    if (new_pwm != controller->pwm_current) {
        int pwm_change = new_pwm - controller->pwm_current;
        int sign = (pwm_change > 0) ? 1 : -1;
        pwm_change = sign * max(abs(pwm_change), controller->min_step);
        new_pwm = controller->pwm_current + pwm_change;
    }
    
    controller->pwm_current = constrain(new_pwm, controller->min_pwm, controller->max_pwm);
}


void performStepResponse() {
    unsigned long elapsedTime = millis() - step_response.startTime;
   
    if (isPositionReached() || elapsedTime >= step_response.maxDuration) {
        if (step_response.isInitialStep) {
            // Switch to final step
            servo_controller.pwm_target = step_response.finalStep;
            step_response.isInitialStep = false;
        } else {
            // Completed one full iteration
            step_response.currentIteration++;
            
            if (step_response.currentIteration < step_response.totalIterations) {
                // Start next iteration
                servo_controller.pwm_target = step_response.initialStep;
                step_response.isInitialStep = true;
            } else {
                // End the step response test
                step_response.active = false;
                PI_programStarted = false;
                Serial.println("Step response test completed.");
                // exit(0);
            }
        }
        
        step_response.startTime = millis();
        step_response.withinToleranceTime = 0;
    }
}


bool isPositionReached() {
    int error = abs(mapAnalogToPWM() - servo_controller.pwm_target);
    
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

int readAndFilterPosition() {
    int rawPosition = analogRead(SERVO_CONTROL_PIN);
    
    // Apply low-pass filter
    filteredPosition = alpha * rawPosition + (1 - alpha) * filteredPosition;
    
    return round(filteredPosition);
    // return rawPosition;
}

int mapAnalogToPWM() {
  int analogValue = readAndFilterPosition();
  return map(analogValue, 863, 308, 2296, 1059);
  // return myservo.readMicroseconds();
}

void startStepResponse() {
    step_response.active = true;
    step_response.startTime = millis();
    step_response.withinToleranceTime = 0;
    step_response.currentIteration = 0;
    step_response.isInitialStep = true;
    servo_controller.pwm_target = step_response.initialStep;
}

void packageServoData(message *msgOut, uint16_t currentPosition) {
  msgOut->opcode = SERVO_ANSWER;
  msgOut->dataLength = 2;

  memcpy(&msgOut->data[0], &currentPosition, sizeof(uint16_t));
  
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
  positionDesired += angleStep / speed_setting;
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

  if ((currentPosition > zeroServoAngle) || (positionDesired > zeroServoAngle)) {
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
  float tempAverage = 0.0;
  float tempAverage1 = 0.0;
  float tempAverage2 = 0.0;
  switch (control_mode)
  {
  case BI_EMG_STREAM:
    tempAverage = data->emgAvg[0];
    biThresholdControl(tempAverage, emgThresholdUpper, emgThresholdLower);
    break;

  case FORCE_STREAM:
    tempAverage = data->forceAvg;
    biThresholdControl(tempAverage, forceThresholdUpper, forceThresholdLower);
    break;
  
  case MONO_EMG_STREAM:
    tempAverage1 = data->emgAvg[0];
    tempAverage2 = data->emgAvg[1];
    monoThresholdBicontrol(tempAverage1,tempAverage2, emgThreshold);
    break;
  
  default:
    // biThresholdControl();
    break;
  }
  
}