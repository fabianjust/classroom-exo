
#include "battery_management.h"

float bat_mvolt = 0;
BatteryVoltage myBat;


void initBatteryVoltage(BatteryVoltage *data) {
  memset(data, 0, sizeof(BatteryVoltage));
}

void initializeBatteryBuffer(BatteryVoltage *bat_data) {
    // Initialize all data structures
    initBatteryVoltage(bat_data);
    
    // Fill buffer with initial readings
    for (int i = 0; i < BAT_WINDOW_SIZE; i++) {
        float voltage = readBatteryVoltage();
        bat_data->samples[i] = voltage;
        bat_data->sum += voltage;
        delay(10); // Small delay between readings
    }
    
    // Set initial average
    bat_data->batAverag = bat_data->sum / BAT_WINDOW_SIZE;
    bat_data->isFull = true;
    bat_data->lastPollTime = millis();
    
    #ifdef DEBUG_BAT
        Serial.print("Initial battery average: ");
        Serial.println(bat_data->batAverag);
    #endif
}

void pollBatteryVoltage(BatteryVoltage* bat_mvolt) {
  unsigned long currentTime = millis();
  if ((currentTime - bat_mvolt->lastPollTime >= POLL_INTERVALL)) {
    float bat_voltage = readBatteryVoltage();
    updateMovingAverage(bat_mvolt, bat_voltage);
    #ifdef DEBUG_BAT
      Serial.print("Intervall:  ");
      Serial.println(currentTime - bat_mvolt->lastPollTime);
  #endif
    bat_mvolt->lastPollTime = currentTime;
    if(MatlabConnected && (!sensor_stream && !pid_stream && !PI_programStarted && !zero_servo)){
      if ((bat_mvolt->batAverag <= HIGH_BAT_LIMIT_WARNING) && (bat_mvolt->batAverag > MED_BAT_LIMIT_WARNING))
      {
        changeLEDButtonColor(0x06); // set LED to yellow
        #ifdef DEBUG_BAT
            Serial.print("LED Color: yellow ");
        #endif
      }
      else if ((bat_mvolt->batAverag <= MED_BAT_LIMIT_WARNING) && (bat_mvolt->batAverag > LOW_BAT_LIMIT_WARNING))
      {
        changeLEDButtonColor(0x07); // set LED to orange
        #ifdef DEBUG_BAT
            Serial.print("LED Color: orange ");
        #endif
      }
      else if (bat_voltage <= LOW_BAT_LIMIT_WARNING)
      {
        changeLEDButtonColor(0x05); // set LED to red
        #ifdef DEBUG_BAT
            Serial.print("LED Color: red ");
        #endif
      }
      else {
        changeLEDButtonColor(0x03); // set LED to green
        #ifdef DEBUG_BAT
            Serial.print("LED Color: green ");
        #endif
      }
    }

  #ifdef DEBUG_BAT
      Serial.print("Bat mV:  ");
      Serial.println(bat_mvolt->batAverag);
  #endif
  }
}

void updateMovingAverage(BatteryVoltage *bv, float newVoltage) {
    // Subtract the oldest sample from the sum
    bv->sum -= bv->samples[bv->currentIndex];
    bv->sum += newVoltage;
    bv->samples[bv->currentIndex] = newVoltage;
    bv->currentIndex = (bv->currentIndex + 1) % BAT_WINDOW_SIZE;
    
    // Check if we've filled the array
    if (bv->currentIndex == 0) {
        bv->isFull = true;
    }
    
    // Calculate the average
    if (bv->isFull) {
        bv->batAverag = bv->sum / BAT_WINDOW_SIZE;
    } else {
        bv->batAverag = bv->sum / (bv->currentIndex + 1);
    }
}


float readBatteryVoltage() {
  float bat_volt = analogRead(BATTERY_VOLTAGE_PIN) * (9.0/4.0);
  return bat_volt;
}