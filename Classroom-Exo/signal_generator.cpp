#include "signal_generator.h"

// Private function declarations
static void initStep(SignalGenerator_t* sig);
static void initRamp(SignalGenerator_t* sig);
static void initSine(SignalGenerator_t* sig);
static float generateStepSignal(SignalGenerator_t* sig);
static float generateRampSignal(SignalGenerator_t* sig);
static float generateSineSignal(SignalGenerator_t* sig);
static void printStateChange(const char* oldState, const char* newState);
static const char* getStateString(SignalGenerator_t* sig);
static bool isPositionReached(SignalGenerator_t* sig, PIDController_t* controller);

// Global variables
SignalGenerator_t signalGen;  // Declaration of global variable


void SignalGenerator_init(SignalGenerator_t* sig, SignalType type) {
    sig->type = type;
    sig->active = false;
    sig->currentIteration = 0;
    sig->totalIterations = 3;
    sig->isInitialStep = true;
    sig->rampingUp = true;
    sig->holding = false;
    sig->isWaiting = false;
    
    switch(type) {
        case STEP:
            initStep(sig);
            break;
        case RAMP:
            initRamp(sig);
            break;
        case COSINE:
            initSine(sig);
            break;
    }
}

static void initStep(SignalGenerator_t* sig) {
    sig->initialValue = 50;
    sig->finalValue = 200;
    sig->maxDuration = 3000;
    sig->holdTime = 0;
}

static void initRamp(SignalGenerator_t* sig) {
    sig->initialValue = 50;
    sig->finalValue = 200;
    unsigned long quarterPeriod = sig->maxDuration / 4;  // Time for ramp up/down
    
    // Calculate ramp rate (units per millisecond)
    float amplitude = sig->finalValue - sig->initialValue;
    sig->rampRate = amplitude / quarterPeriod;  // This gives us the correct rate to reach finalValue in quarterPeriod
    
    
    
    // Calculate and print estimated cycle time
    unsigned long rampTime = (sig->finalValue - sig->initialValue) / sig->rampRate;
    Serial.print(F("Estimated cycle time: "));
    Serial.print((rampTime * 2 + sig->holdTime) / 1000.0);
    Serial.println(F(" seconds"));

    sig->rampingUp = true;
    sig->rampingDown = false;
    sig->holding = false;
    sig->holdingLow = false;
}

static void initSine(SignalGenerator_t* sig) {
    sig->amplitude = 75;
    sig->frequency = 0.2;
    sig->offset = 128;
    sig->maxDuration = 10000;
    sig->holdTime = 0;
}

void SignalGenerator_setHoldTime(SignalGenerator_t* sig, unsigned long holdTime) {
    sig->holdTime = holdTime;
}

void SignalGenerator_setRampRate(SignalGenerator_t* sig, float rate) {
    sig->rampRate = rate;
}

void SignalGenerator_setAmplitude(SignalGenerator_t* sig, float amp) {
    sig->amplitude = amp;
}

void SignalGenerator_setFrequency(SignalGenerator_t* sig, float freq) {
    sig->frequency = freq;
}

void SignalGenerator_setRange(SignalGenerator_t* sig, float initial, float final) {
    sig->initialValue = initial;
    sig->finalValue = final;
}

void SignalGenerator_setIterations(SignalGenerator_t* sig, int iterations) {
    sig->totalIterations = iterations;
}

void SignalGenerator_setWaitTime(SignalGenerator_t* sig, unsigned long waitTime) {
    sig->waitDuration = waitTime;
}

void SignalGenerator_setStepValues(SignalGenerator_t* sig, float initialStep, float finalStep) {
    sig->initialStep = initialStep;
    sig->finalStep = finalStep;
}

void SignalGenerator_setMaxDuration(SignalGenerator_t* sig, unsigned long duration) {
    sig->maxDuration = duration;
}

void SignalGenerator_moveToNextStep(SignalGenerator_t* sig, PIDController_t* controller) {
    if (sig->currentIteration >= sig->totalIterations) {
        sig->isWaiting = true;
        sig->waitStartTime = millis();
        Serial.println(F("Starting final wait period"));
        return;
        
    }

    if (sig->isInitialStep) {
        // Currently at initial, move to final
        controller->setpoint_target = sig->finalValue;
        sig->isInitialStep = false;
        Serial.println(F("#############################Moving to final position##########################"));
    } else {
        // Currently at final, move to initial and increment iteration
        controller->setpoint_target = sig->initialValue;
        sig->isInitialStep = true;
        sig->currentIteration++;
        Serial.println(F("######################Moving to initial position################"));
    }
   
    sig->startTime = millis();
    sig->withinToleranceTime = 0;
}

void SignalGenerator_end(SignalGenerator_t* sig, PIDController_t* controller) {
    sig->active = false;
    
    // Reset controller to default values
    controller->pwm_target = 1500;
    controller->pwm_current = 1500;
    controller->pwm_calculated = 0;
    controller->current_angle = 0.0;
    controller->setpoint_target = 0.0;
    controller->pid_output = 0.0;
    controller->pid_input = 0.0;
    controller->velocity_current = 0;
    controller->integral_error = 0;
    
    Serial.println(F("Signal response test completed."));
    // send stop bit
    msg2send.header = HEADER;
    msg2send.opcode = RESP_PID_FINISHED;
    msg2send.dataLength = 0;
    sendMessage(&msg2send);
}

static bool isPositionReached(SignalGenerator_t* sig, PIDController_t* controller) {
    Serial.print("is pos reached: ");
    Serial.println(abs(controller->setpoint_target - controller->current_angle) <= controller->deadband);
    return abs(controller->setpoint_target - controller->current_angle) <= controller->deadband;
}

void SignalGenerator_update(SignalGenerator_t* sig, PIDController_t* controller) {
    if (!sig->active) return;
    
    unsigned long currentTime = millis();
    

    // Handle waiting period (both initial and final)
    if (sig->isWaiting) {
        if (currentTime - sig->waitStartTime >= sig->waitDuration) {
            sig->isWaiting = false;
            controller->integral_error = 0;
            // controller->setpoint_target = sig->initialValue;
            
            // If we were in final wait (iterations completed), end the signal
            if (sig->currentIteration >= sig->totalIterations) {
                SignalGenerator_end(sig, controller);
                return;
            }
            
            // Otherwise it was initial wait, continue with signal
            sig->startTime = currentTime;
            Serial.println(F("  --- waiting done"));
        }
        Serial.print(F("waiting "));
        return;
    }

    unsigned long elapsedTime = currentTime - sig->startTime;
    
    switch(sig->type) {
        case STEP: {
            // For step response: If we just finished waiting, move to final value immediately
            if (sig->isInitialStep && (sig->currentIteration == 0)) {  // Just finished waiting
                controller->integral_error = 0;
                SignalGenerator_moveToNextStep(sig, controller);
            }
            // Normal timing check for subsequent moves
            else if (elapsedTime >= sig->maxDuration) {
                Serial.print("  elapsedTime:  ");
                Serial.print(elapsedTime);
                Serial.print("  maxDuration:  ");
                Serial.println(sig->maxDuration);
                controller->integral_error = 0;
                SignalGenerator_moveToNextStep(sig, controller);
            }
            break;
        }
        
        case RAMP: {
            float targetValue = generateRampSignal(sig);
            controller->setpoint_target = targetValue;
            break;
        }
        
        case COSINE: {
            float targetValue = generateSineSignal(sig);
            controller->setpoint_target = targetValue;
            
            if (elapsedTime >= sig->maxDuration) {
                sig->currentIteration++;
                if (sig->currentIteration >= sig->totalIterations) {
                    // Instead of ending immediately, start final wait period
                    sig->isWaiting = true;
                    sig->waitStartTime = currentTime;
                    Serial.println(F("Starting final wait period for sine"));
                } else {
                    sig->startTime = currentTime;
                }
            }
        }break;
    }
}

void SignalGenerator_start(SignalGenerator_t* sig) {
    sig->active = true;
    sig->startTime = millis();
    sig->currentIteration = 0;
    sig->rampingUp = true;
    sig->holding = false;
    sig->isWaiting = true;
    sig->waitStartTime = sig->startTime;
}

void SignalGenerator_stop(SignalGenerator_t* sig) {
    sig->active = false;
}

void SignalGenerator_reset(SignalGenerator_t* sig) {
    memset(sig, 0, sizeof(SignalGenerator_t));
    sig->type = STEP;
}

bool SignalGenerator_isActive(SignalGenerator_t* sig) {
    return sig->active;
}

float SignalGenerator_generateSignal(SignalGenerator_t* sig) {
    if (!sig->active) return sig->initialValue;
    
    switch(sig->type) {
        case STEP:
            return generateStepSignal(sig);
        case RAMP:
            return generateRampSignal(sig);
        case COSINE:
            return generateSineSignal(sig);
        default:
            return sig->initialValue;
    }
}

static float generateStepSignal(SignalGenerator_t* sig) {
    return sig->isInitialStep ? sig->initialValue : sig->finalValue;
}

static float generateRampSignal(SignalGenerator_t* sig) {
    unsigned long currentTime = millis();
    float target = sig->initialValue;
    
    // If we're still waiting, return initial value
    if (sig->isWaiting) {
        return sig->initialValue;
    }
    
    unsigned long elapsedTime = currentTime - sig->startTime;
    
    if (sig->rampingUp) {
        // Calculate target based on ramp rate and elapsed time
        float change = sig->rampRate * elapsedTime;
        target = sig->initialValue + change;
       
        // Debug print to see what's happening
        Serial.print("Elapsed: ");
        Serial.print(elapsedTime);
        Serial.print(" Change: ");
        Serial.print(change);
        Serial.print(" Target: ");
        Serial.println(target);
       
        if (target >= sig->finalValue) {
            target = sig->finalValue;
            sig->rampingUp = false;
            sig->holding = true;
            sig->holdStartTime = currentTime;
            printStateChange("RAMP_UP", "HOLDING_HIGH");
        }
    }
    else if (sig->holding) {
        target = sig->finalValue;
        unsigned long holdElapsedTime = currentTime - sig->holdStartTime;
       
        if (holdElapsedTime >= sig->holdTime) {
            sig->holding = false;
            sig->rampingDown = true;
            sig->startTime = currentTime;  // Reset start time for ramp down
            printStateChange("HOLDING_HIGH", "RAMP_DOWN");
        }
    }
    else if (sig->rampingDown) {
        float change = sig->rampRate * elapsedTime;
        target = sig->finalValue - change;
        // Debug print to see what's happening
        Serial.print("Elapsed: ");
        Serial.print(elapsedTime);
        Serial.print(" Change: ");
        Serial.print(change);
        Serial.print(" Target: ");
        Serial.println(target);

        if (target <= sig->initialValue) {
            target = sig->initialValue;
            sig->rampingDown = false;
            sig->holdingLow = true;
            sig->holdStartTime = currentTime;
            printStateChange("RAMP_DOWN", "HOLDING_LOW");
        }
    }
    else if (sig->holdingLow) {
        target = sig->initialValue;
        unsigned long holdElapsedTime = currentTime - sig->holdStartTime;
       
        if (holdElapsedTime >= sig->holdTime) {
            sig->holdingLow = false;
            sig->rampingUp = true;
            sig->startTime = currentTime;
            sig->currentIteration++;
           
            if (sig->currentIteration >= sig->totalIterations) {
                // Start final wait period
                sig->isWaiting = true;
                sig->waitStartTime = currentTime;
                Serial.println(F("Starting final wait period"));
            } else {
                printStateChange("HOLDING_LOW", "RAMP_UP");
            }
        }
    }
    
    return constrain(target, sig->initialValue, sig->finalValue);
}

static float generateSineSignal(SignalGenerator_t* sig) {
    unsigned long elapsedTime = millis() - sig->startTime;
    float timeInSeconds = elapsedTime / 1000.0;
    return sig->offset + sig->amplitude/2 * cos(2 * PI * sig->frequency * timeInSeconds + PI);
}


static void printStateChange(const char* oldState, const char* newState) {
    Serial.print(F("State change: "));
    Serial.print(oldState);
    Serial.print(F(" -> "));
    Serial.println(newState);
}

static const char* getStateString(SignalGenerator_t* sig) {
    if (!sig->active) return "INACTIVE";
    if (sig->isWaiting) return "WAITING";
    
    switch(sig->type) {
        case STEP:
            return sig->isInitialStep ? "INITIAL" : "FINAL";
        case RAMP:
            if (sig->holding) return "HOLDING";
            return sig->rampingUp ? "RAMP_UP" : "RAMP_DOWN";
        case COSINE:
            return "COSINE";
        default:
            return "UNKNOWN";
    }
}

void SignalGenerator_printStatus(SignalGenerator_t* sig, PIDController_t* controller) {
    Serial.print(millis());
    Serial.print(F(","));
    Serial.print(controller->setpoint_target);
    Serial.print(F(","));
    Serial.print(controller->pid_output);
    Serial.print(F(","));
    Serial.print(controller->pid_input);
    Serial.print(F(","));
    Serial.print(getStateString(sig));
    
    if (sig->isWaiting) {
        Serial.print(F(" (Waiting "));
        Serial.print(millis() - sig->waitStartTime);
        Serial.print(F("/"));
        Serial.print(sig->waitDuration);
        Serial.print(F("ms)"));
    }
    else if (sig->type == RAMP && sig->holding) {
        Serial.print(F(" (Holding "));
        Serial.print(millis() - sig->holdStartTime);
        Serial.print(F("/"));
        Serial.print(sig->holdTime);
        Serial.print(F("ms)"));
    }
    
    Serial.print(F(",Cycle:"));
    Serial.print(sig->currentIteration + 1);
    Serial.print(F("/"));
    Serial.println(sig->totalIterations);
}

int SignalGenerator_getCurrentIteration(SignalGenerator_t* sig) {
    return sig->currentIteration;
}