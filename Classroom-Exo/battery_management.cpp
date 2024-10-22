
#include "battery_management.h"

float bat_mvolt = 0;
BatteryVoltage myBat;


void initBatteryVoltage(BatteryVoltage *data) {
  memset(data, 0, sizeof(BatteryVoltage));
}

void pollBatteryVoltage(BatteryVoltage* bat_mvolt) {
  unsigned long currentTime = millis();
  if ((currentTime - bat_mvolt->lastPollTime >= POLL_INTERVALL)) {
    float bat_voltage = readBatteryVoltage();
    updateMovingAverage(bat_mvolt, bat_voltage);
    bat_mvolt->lastPollTime = currentTime;
    if(MatlabConnected && (!sensor_stream || !pid_stream || !zero_servo)){
      if ((bat_mvolt->batAverag <= HIGH_BAT_LIMIT_WARNING) && (bat_mvolt->batAverag > MED_BAT_LIMIT_WARNING))
      {
        changeLEDButtonColor(0x06); // set LED to light green
      }
      else if ((bat_mvolt->batAverag <= MED_BAT_LIMIT_WARNING) && (bat_mvolt->batAverag > LOW_BAT_LIMIT_WARNING))
      {
        changeLEDButtonColor(0x07); // set LED to orange
      }
      else if (bat_voltage <= LOW_BAT_LIMIT_WARNING)
      {
        changeLEDButtonColor(0x05); // set LED to red
      }
      else 
        changeLEDButtonColor(0x03); // set LED to green
    }

  #ifdef DEBUG2
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
  float bat_volt = analogRead(BATTERY_VOLTAGE_PIN) * 3 * 3;
  return bat_volt;
}