/*******************************************************************************
  #                                                                              #
  # Using the Firebeetle 2 ESP32-E as battery powered Lumin sensor               #
  # Project:                                                                     #
  #                                                                              #
  # Firebeetle documentation at:                                                 #
  # https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654                #
  #                                                                              #
  # Copyright (C) 2022 Neil Glessner                                             #
  #                                                                              #
  # This program is free software; you can redistribute it and/or modify         #
  # it under the terms of the GNU General Public License as published by         #
  # the Free Software Foundation; version 2 of the License.                      #
  #                                                                              #
  # This program is distributed in the hope that it will be useful,              #
  # but WITHOUT ANY WARRANTY; without even the implied warranty of               #
  # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
  # GNU General Public License for more details.                                 #
  #                                                                              #
  # You should have received a copy of the GNU General Public License            #
  # along with this program; if not, write to the Free Software                  #
  # Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
  #                                                                              #
********************************************************************************/
#include <Adafruit_TSL2591.h>
#include <Adafruit_Sensor.h>
#include "fdrs_sensor_config.h"
#include <fdrs_sensor.h>
#include "esp_adc_cal.h"
#include "DHT.h"

#define LOW_BATTERY_VOLTAGE 3.20
#define VERY_LOW_BATTERY_VOLTAGE 3.10
#define CRITICALLY_LOW_BATTERY_VOLTAGE 3.00

#define DHTPIN 4
#define DHTTYPE DHT22               // DHT21, DHT11
#define LUX_SENSITIVITY 15           // the minimum LUX change we transmit, skip otherwise, 0 = always
#define POLLING_DURATION 60         //how often we check light intensity in seconds
#define USE_STATUS_LEDS false       // use LED to indicate awake processing
#define MINIMUM_TRANSMIT_CYCLES 30  // trasmit at least every x cycles

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
DHT dht(DHTPIN, DHTTYPE);

// cached values - lifetime reset only by power cycle
RTC_NOINIT_ATTR struct {
  uint64_t NumberOfRestarts;     //number of restarts
  uint64_t LastTransmitRestart;  //the restart count during last transmit
  int LastReportedLux;           //keep track of last reported light value
} cache;

// Pinout setup
// v+ to TSL2591
uint16_t SENSOR_POWER_PIN = 12;
uint16_t STATUS_LED_PIN = 2;

/******************************************************************************
  Description.: since this is a battery sensor, everything happens in setup
              and when it is done the device enters deep-sleep
******************************************************************************/
void setup() {
  //stuff to do every run
  //setup pinouts and freq
  Serial.begin(115200);
  setCpuFrequencyMhz(80);
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  if (USE_STATUS_LEDS) {
    pinMode(STATUS_LED_PIN, OUTPUT);
  }

  Serial.print("===================================================\r\n");

  if (esp_reset_reason() == ESP_RST_POWERON) {
    //stuff to do only once
    Serial.printf("ESP was just switched ON\r\n");
    cache.NumberOfRestarts = 0;
    cache.LastReportedLux = 0;
    cache.LastTransmitRestart = 0;
    sleepFDRS(1);
  } else {
    if (USE_STATUS_LEDS) {
      digitalWrite(STATUS_LED_PIN, HIGH);
    }
    cache.NumberOfRestarts++;
    Serial.printf("Number of Restarts: %d\r\n", cache.NumberOfRestarts);

    ReportLux();
  }

  // turn off activity LED and go to deep sleep
  if (USE_STATUS_LEDS) {
    digitalWrite(STATUS_LED_PIN, LOW);
  }
  Serial.printf("=== entering deepsleep after %d ms ===\r\n", millis());
  sleepFDRS(POLLING_DURATION);  //Goodnight

  Serial.println("This should never get printed");
}

void loop() {
  Serial.println("This loop message should never get printed");
}

/******************************************************************************
  Description.: reads the battery voltage through the voltage divider at GPIO34
              if the ESP32-E has calibration eFused those will be used.
              In comparison with a regular voltmeter the values of ESP32 and
              multimeter differ only about 0.05V
  Return Value: battery voltage in volts
******************************************************************************/
float readBattery() {
  uint32_t value = 0;
  int rounds = 11;
  esp_adc_cal_characteristics_t adc_chars;

  //battery voltage divided by 2 can be measured at GPIO34, which equals ADC1_CHANNEL6
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  //to avoid noise, sample the pin several times and average the result
  for (int i = 1; i <= rounds; i++) {
    value += adc1_get_raw(ADC1_CHANNEL_6);
  }
  value /= (uint32_t)rounds;

  //due to the voltage divider (1M+1M) values must be multiplied by 2
  //and convert mV to V
  return esp_adc_cal_raw_to_voltage(value, &adc_chars) * 2.0 / 1000.0;
}

/******************************************************************************
  Description.: send message
******************************************************************************/
bool ReportLux() {
  // give the sensor to 8ms to wake up
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(8);
  int vis = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  digitalWrite(SENSOR_POWER_PIN, LOW);

  int diff = abs(cache.LastReportedLux - vis);
  Serial.printf("Cached: %d ", cache.LastReportedLux); Serial.printf("Full: %d ", vis); Serial.printf("Diff: %d\r\n", diff);

  // transmit only if one of the following is true
  // 1) light intensity has changed more than the LUX_SENSITIVITY threshold
  // 2) light intensity is now zero and wasn't previously 0
  // 3) it's been X cycles since we last transmitted
  if (diff >= LUX_SENSITIVITY || (cache.LastReportedLux != 0 && vis == 0) || cache.LastTransmitRestart + MINIMUM_TRANSMIT_CYCLES <= cache.NumberOfRestarts) {
    Serial.println(F(" --- New Light Value ---  "));
    cache.LastReportedLux = vis;

    beginFDRS();
    dht.begin();
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float b = readBattery();

    Serial.printf("Temp: (%f degC)\r\n", t);
    Serial.printf("Humidity: (%f RH)\r\n", h);
    Serial.printf("Battery Voltage: (%f V)\r\n", b);

    loadFDRS(b, VOLTAGE_T);
    loadFDRS(vis, LIGHT_T);

    if (isnan(h) || isnan(t)) {
      DBG("Failed to read from DHT sensor!");
    } else {
      loadFDRS(h, HUMIDITY_T);
      loadFDRS(t, TEMP_T);
    }

    sendFDRS();
    cache.LastTransmitRestart = cache.NumberOfRestarts;
  } else {
    Serial.println(F(" --- No Change ---  "));
  }

  return false;
}