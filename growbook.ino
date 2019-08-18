/**

      2 * DS1822+   https://datasheets.maximintegrated.com/en/ds/DS1822.pdf  9-12bit  -55-125
      1 * nodeMCU
      1 * Solid state Relay FOTEK SSR-25 DA
      1 * 220v to 5v USB PS
    Security:
      1. On WiFi disconnect the load will be disabled(KEEP mode only)
      2. Delay between disable to enable- default 60 seconds(KEEP mode only)
      3. In case temperature value is anomalous the load will be disabled
      5. Check Internet connectivity, if there is no ping to 8.8.8.8, load will be disabled
      6. Add reconnect after X pings failures
      7. Added \_FLAG_FORCE_TMP_CHECK, When true will cause to immediately check, used also when thermometer wire were troubled

*/
#include "Arduino.h"
#include "Esp.h"
#include <NTPClient.h>
#include <Wire.h>


#ifdef ESP32

#include <WiFi.h>
#include <ESP32Ping.h>
//#include <WebServer.h>
#include <ESPmDNS.h>
#include "ESP32httpUpdate.h"
#include <HTTPClient.h>

#else

#include <ESP8266WiFi.h>
#include <ESP8266Ping.h>
//#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "ESP8266httpUpdate.h"
#include <ESP8266HTTPClient.h>

#endif


#include <WiFiClient.h>

#include "FS.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHTesp.h"
#include <Ticker.h>

/*******************************************************************************************************/
#define   VERSION                           0.44
// 0.44 Production release, 2019.08.17_18:25
// 0.42 Production release, 2019.08.17_14:55
// 0.38 Production release, 2019.08.17_14:15
// 0.37 Production release, 2019.08.16_15:55  Devide code to files, improve DHT detection
// 0.36 Production release, 2019.08.15_13:30  Improve stability
// 0.35 Production release, 2019.08.14_11:45
// 0.33 Production release, 2019.08.10_15:14
// 0.32 Production release, 2019.08.04_18:29

// WiFi settings
#define   WIFI_SSID                         "Radiation"
#define   WIFI_PASS                         "polkalol"

// NTP settings
#define   NTP_SERVER                        "1.asia.pool.ntp.org"   // Pool of ntp server http://www.pool.ntp.org/zone/asia
#define   NTP_TIME_OFFSET_SEC               10800                   // Time offset
#define   NTP_UPDATE_INTERVAL_MS            60000                   // NTP Update interval - 1 min

// Unchangeable settings
#define   INCORRECT_EPOCH                   200000                  // Minimum value for time epoch
#define   HIGH_TEMPERATURE                  85                      // If temperature bigger of this value we recheck it again
#define   LOW_TEMPERATURE                   -55
/*******************************************************************************************************/
// Thermometer, hydrometer, humidity light sensor and wire settings

#ifdef ESP32

#define   HYDROMETER_PIN                    A0
#define   LIGHT_SENSOR_D0                   16
#define   HYDROMETER_D0_PIN                 5
#define   ONE_WIRE_BUS                      16                      // D4 2
#define   DHT_HUMIDITY_PIN                  14                      // 14 D5 of NodeMCU is GPIO14

#else

#define   HYDROMETER_PIN                    A0
#define   LIGHT_SENSOR_D0                   D0
#define   HYDROMETER_D0_PIN                 D1
#define   ONE_WIRE_BUS                      D4                      // D4 2
#define   DHT_HUMIDITY_PIN                  D5                      // 14 D5 of NodeMCU is GPIO14

#endif



#define   TEMPERATURE_PRECISION             12                      // Possible value 9-12
#define   NUMBER_OF_SENSORS                 4

// UART
#define   UART_BAUD_RATE                    115200 //921600

// GENERAL
#define   LOOP_DELAY                        50                      // Wait each loop for LOOP_DELAY
#define   CHECK_INTERNET_CONNECT            1                       // For disable internet connectiviy check use 0
#define   RECONNECT_AFTER_FAILS             100                     // 20 = ~1 min -> 100 =~ 4min
#define   MESSAGE_OPT                       1

#define   MIN_TEMPERATURE_TH                0.09                    // Minimal threshhold for temperature to update
#define   MIN_HUMIDITY_TH                   0.1                     // Minimal threshhold for humidity to update
/**
   shows counter values identical to one second
   For example loop_delay=10, counter sec will be 100 , when (counter%100 == 0) happens every second
*/
#define COUNTER_IN_LOOP_SECOND              (int)(1000/LOOP_DELAY)
#define CHECK_HUMIDITY_COUNTER              (COUNTER_IN_LOOP_SECOND*30*4) // every 5 seconds * 4
#define CALL_SERVER_COUNTER                 (COUNTER_IN_LOOP_SECOND*10)
#define CHECK_SENSORS                       (COUNTER_IN_LOOP_SECOND*20) //(COUNTER_IN_LOOP_SECOND*3)

#define CHECK_SENSORS_1                     ((COUNTER_IN_LOOP_SECOND+1)*10)
#define CHECK_SENSORS_2                     (COUNTER_IN_LOOP_SECOND*22+2)
#define CHECK_SENSORS_3                     (COUNTER_IN_LOOP_SECOND*23+3)
#define CHECK_SENSORS_4                     (COUNTER_IN_LOOP_SECOND*27+4)
#define CHECK_SENSORS_5                     (COUNTER_IN_LOOP_SECOND*31+5)
#define CHECK_SENSORS_6                     (COUNTER_IN_LOOP_SECOND*49+6)
#define CHECK_SENSORS_7                     (COUNTER_IN_LOOP_SECOND*41+7)
#define CHECK_SENSORS_8                     (COUNTER_IN_LOOP_SECOND*51+8)

#define NTP_UPDATE_COUNTER                  (COUNTER_IN_LOOP_SECOND*60*3)
#define CHECK_INTERNET_CONNECTIVITY_CTR     (COUNTER_IN_LOOP_SECOND*120)

//#define GROWBOOK_URL                        "http://192.168.1.206:8082/"
#define GROWBOOK_URL                        "http://growbook.anshamis.com/"


// INTERRUPT
#define INTERRUPT_TIME                      120000000                //  600000      //12us
/*******************************************************************************************************/
#include "structs.h"
const char          *ssid                     = WIFI_SSID;
const char          *password                 = WIFI_PASS;
int                 counter                   = 0;
int                 last_disable_epoch        = 0;
bool                internet_access           = 0;
unsigned short      internet_access_failures  = 0;
String              sensorsSingleLog          = "";
/**
 ****************************************************************************************************
*/
#ifdef ESP32

hw_timer_t * timer = NULL;
//WebServer    server(80);
#else

//ESP8266WebServer    server(80);
#endif

OneWire               oneWire(ONE_WIRE_BUS);
DallasTemperature     sensor(&oneWire);
DeviceAddress         insideThermometer[NUMBER_OF_SENSORS];       // arrays to hold device address
IPAddress             pingServer (8, 8, 8, 8);    // Ping server
DHTesp                dhtMain;
HTTPClient            httpClient;    //Declare object of class HTTPClient
WiFiClient            wifi_client;
WiFiUDP               ntpUDP;
NTPClient             timeClient(ntpUDP, NTP_SERVER, NTP_TIME_OFFSET_SEC, NTP_UPDATE_INTERVAL_MS);
/** You can specify the time server pool and the offset (in seconds, can be changed later with setTimeOffset()).
    Additionaly you can specify the update interval (in milliseconds, can be changed using setUpdateInterval()). */

float                           current_temp[NUMBER_OF_SENSORS];
float                           temp_sum_value_prev = 0;
float                           temp_min_value_prev = 0;
float                           temp_max_value_prev = 0;
float                           hydro_value_prev = -1;
float                           humidity_value_prev = 0;
float                           humidity_value = 0;
bool                            light_enabled = false;
bool                            reset_info_posted = false;
volatile unsigned long          boot_time = 0;
volatile unsigned long          uptime = 0;
volatile bool                   update_time_flag   = false;
volatile bool                   firmwareUpdateFlag = false;
/*******************************************************************************************************/

#include "functions.h"
#include "interrupts.h"

void setup(void)
{
  pinMode(LIGHT_SENSOR_D0, INPUT);
  sensor.begin();
  Serial.begin(UART_BAUD_RATE);
  if (CHECK_INTERNET_CONNECT) {
    internet_access = 0;
  } else {
    internet_access = 1;
  }
  Serial.println("");
  wifi_connect();
  message("Serial communication started.", PASS);
  print_reset_info();
  message("Starting SPIFFS....", INFO);
  if (!SPIFFS.begin()) {
    message("Failed to init SPIFFS ", CRITICAL);
    message("Format SPIFFS ", CRITICAL);
    SPIFFS.format();
    message("Starting SPIFFS....", INFO);
    SPIFFS.begin();
  }
  message("SPIFFS startted.", PASS);
  timeClient.begin();
  delay(1000);
  dhtMain = startSensor(dhtMain, DHT_HUMIDITY_PIN);
  start_thermal();
  //timeClient.update();

  timeClient.forceUpdate();
  delay(1000);
  message(" ----> All started <----", PASS);
  delay(1000);
  wifi_check();
  print_all_info();

#ifdef ESP8266

  ESP.wdtEnable(10000);
  ESP.wdtDisable();
#endif

  delay(1000);
  check_connectivity(true);

  while (counter < 100) {
    if ((CHECK_INTERNET_CONNECT && internet_access) || !CHECK_INTERNET_CONNECT) {
      message(" ----> internet_access is OK<----", PASS);
      break;
    }

#ifdef ESP8266

    ESP.wdtFeed();
#endif

    delay(100);
    counter += 1;
    if (counter == 50) {
      check_connectivity(true);
    }
    message("internet_access: " + String(internet_access) + " CHECK_INTERNET_CONNECT:" + String(CHECK_INTERNET_CONNECT), PASS);
  }

  counter = -100;
  update_time();

#ifdef ESP32

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);// Attach onTimer function to our timer.
  timerAlarmWrite(timer, 1000000, true);      // Set alarm to call onTimer function every second (value in microseconds).Repeat the alarm (third parameter)
  timerAlarmEnable(timer);                    // Start an alarm
#else

  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(INTERRUPT_TIME); //120000 us
#endif
  message(" ----> Start Loop <----", PASS);
}


/**
  /////////////////////// L O O P   F U N C T I O N //////////////////////////
  ////////////////////////////////////////////////////////////////////////////
*/

void loop(void)
{
  if (internet_access && (boot_time == 0 || boot_time < INCORRECT_EPOCH)) {
    boot_time = timeClient.getEpochTime();
    message("Boot time updated...", DEBUG);
    delay(999);
  }

#ifdef ESP8266

  ESP.wdtFeed();
#endif

  wifi_check();

  if (counter % NTP_UPDATE_COUNTER == 0 || counter == -50) {
    if (internet_access) {
      message("Starting update the time...", DEBUG);
      update_time();
    }
  }

  check_light(false);
  if (counter % CHECK_HUMIDITY_COUNTER == 0) {
    read_dht(dhtMain);
  }

  if (counter % 600 == 0) {
    read_cmd_flow();
  }

  if (counter % CHECK_SENSORS_1 == 0) {
    sonsors_dallas();
    check_light(false);
  }

  //  if (counter % CHECK_SENSORS_2 == 0) {
  //    check_light(false);
  //  }
  //  if (counter % CHECK_SENSORS_3 == 0) {
  //
  //  }
  //  if (counter % CHECK_SENSORS_4 == 0) {
  //
  //  }
  if (counter % CHECK_SENSORS_5 == 0) {
    sensors_hydrometer();
    check_light(false);
  }
  if (counter % CHECK_SENSORS_6 == 0) {
    sensors_light();
  }
  if (counter % CHECK_SENSORS_7 == 0) {
    sonsors_dht();
    check_light(false);
  }
  if (counter % CHECK_SENSORS_8 == 0) {

  }

  if (counter % CALL_SERVER_COUNTER == 0 && internet_access) {
    growBookPostValues();

#ifdef ESP8266

    ESP.wdtFeed();
#endif

  }

  check_connectivity(false);
  if (counter == 20) {
    print_all_info();
  }


  check_light(false);
  delay(LOOP_DELAY / 3);
  check_light(false);
  delay(LOOP_DELAY / 3);
  check_light(false);
  delay(LOOP_DELAY / 3);
  counter++;
  if (counter >= 100000) {
    counter = 0;
  }
  //uptime = timeClient.getEpochTime() - boot_time;
}

//-------------------------------------------------------------------------------------------------------------------//
/**

*/
void read_cmd_flow()
{
  String output = read_cmd("");
  if (output.length() > 500 || output.length() < 4) {
    return;
  }
  String key = getValue(output, '^', 0);
  String value = getValue(output, '^', 1);
  if (key.length() > 1)
  {
    message("READ cmd:" + output + " KEY=" + String(key) + " VALUE=" + String(value));
    if (key == "reboot" && value.toInt() == 1) {
      message("Resetting ESP" , WARNING);
      delay(500);
      ESP.restart();
    }
    if (key == "firmwareUpdate") {
      update_firmware(value);
    }
  }
}

void update_firmware(const String &value)
{
  firmwareUpdateFlag = true;
#ifdef ESP32
  timerAlarmDisable(timer);
#endif
  message("Update firmware from " + String(value) , WARNING);
#ifdef ESP8266
  ESP.wdtFeed();
#endif
  ESPhttpUpdate.rebootOnUpdate(true);
#ifdef ESP8266
  ESP.wdtFeed();
#endif
  delay(1000); // Wait till timer is finished
  HTTPUpdateResult ret = ESPhttpUpdate.update(value);
#ifdef ESP8266
  ESP.wdtFeed();
#endif
  message("Resetting ESP" , WARNING);
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      message("[update] HTTP_UPDATE_FAILED Update failed." + String(ESPhttpUpdate.getLastError()) + " " + String(ESPhttpUpdate.getLastErrorString().c_str()));
      break;
    case HTTP_UPDATE_NO_UPDATES:
      message("[update] HTTP_UPDATE_NO_UPDATES Update no Update.");
      break;
    case HTTP_UPDATE_OK:
      message("[update] Update ok."); // may not called we reboot the ESP
      break;
  }
#ifdef ESP8266
  ESP.wdtFeed();
#endif
  interrupts();
  firmwareUpdateFlag = false;
#ifdef ESP32
  timerAlarmEnable(timer);
#else
  timer1_write(INTERRUPT_TIME);//12us
#endif
  delay(500);
}


const bool sensors_hydrometer()
{
  float hydro_value = analogRead(HYDROMETER_PIN);
  float hydro_value_src = hydro_value;
  hydro_value = map(hydro_value, 0, 1024, 1000, 0) / 10.0;
  String model = "HYDRO_" + String(HYDROMETER_PIN) + "_";
  bool epoch_trigger = timeClient.getEpochTime() % 7 == 0;
  if (hydro_value >= 0 && hydro_value <= 100) {
    if (hydro_value != hydro_value_prev || hydro_value_prev == 0 || epoch_trigger) {
      growBookPostEvent(String(hydro_value), model + "_-_" + getNodeName() + String("_-_0"), TypeNames[HYDROMETER], "", "", "");
      hydro_value_prev = hydro_value;
    }
  } else {
    message("Hydrometer bad value: " + String(hydro_value) + " -- SRC:" + String(hydro_value_src), CRITICAL);
  }
  growBookPostValue("hydrometer", String(hydro_value));

  return true;
}

const bool check_light(const bool &verbose)
{
  int digitalVal = digitalRead(LIGHT_SENSOR_D0);    // Read the digital interface
  bool change_found = false;
  if (digitalVal == HIGH) {
    if (light_enabled) {
      change_found = true;
      if (verbose) {
        message("Light is OFF", DEBUG);
      }
    }
    light_enabled = false;
  } else {
    if (!light_enabled) {
      change_found = true;
      if (verbose) {
        message("Light is ON", DEBUG);
      }
    }
    light_enabled = true;
  }

  if (change_found) {
    growBookPostValue("light", String(light_enabled));
    if (verbose) {
      message("Light is OFF", DEBUG);
    }
  }
  return light_enabled;
}
/**

*/
const bool sensors_light()
{

  check_light(true);
  String serail = "LDR_" + getNodeName() + String("_" + String(LIGHT_SENSOR_D0));
  growBookPostValue("light", String(light_enabled));
  //growBookPostEvent(String(light_enabled), serail, TypeNames[LIGHT], "", "", "");
  /// }
  return true;
}

const bool sonsors_dallas()
{
  short int devices_count = sensor.getDeviceCount();
  unsigned short int devices_count_good = 0;
  float sum_tmp = 0;
  temp_max_value_prev = LOW_TEMPERATURE;
  temp_min_value_prev = HIGH_TEMPERATURE;
  for (short int i = 0; i < devices_count; i++) {
    float prevTmp = current_temp[i];
    float tmp_1 = 0;
    float tmp_2 = 1;
    current_temp[i] = getTemperature(sensor, i);
    if (prevTmp != current_temp[i]) {
      short int _c = 0;
      while (tmp_1 != tmp_2 && _c < 10) {
        _c++;
        delay(20 * _c);
#ifdef ESP8266
        ESP.wdtFeed();
#endif
        tmp_1 = getTemperature(sensor, i);
        tmp_2 = getTemperature(sensor, i);
        if (tmp_1 > HIGH_TEMPERATURE && tmp_1 == tmp_2) {
          message("High temperature found, recheck. Current" + String(tmp_1) + " Threshold[HIGH_TEMPERATURE]: " + String(HIGH_TEMPERATURE), CRITICAL);
          tmp_1 = 0;
          delay(20);
#ifdef ESP8266
          ESP.wdtFeed();
#endif
        }
      }
      current_temp[i] = tmp_1;
    }

    float tmp_diff = prevTmp - current_temp[i];
    if (current_temp[i] > LOW_TEMPERATURE && current_temp[i] < HIGH_TEMPERATURE) {
      sum_tmp += current_temp[i];
      devices_count_good += 1;
      temp_max_value_prev = max(temp_max_value_prev, current_temp[i]);
      temp_min_value_prev = min(temp_min_value_prev, current_temp[i]);
      bool epoch_trigger = timeClient.getEpochTime() % 7 == 0;
      if (fabs(tmp_diff) > MIN_TEMPERATURE_TH || epoch_trigger) {
#ifdef EPOCH_TRIGGER_DEBUG
        if (epoch_trigger) {
          message("- ---          ****************************** ---------- EPOCH TRIGGER");
        }
#endif
        growBookPostEvent(String(current_temp[i]), String(getAddressString(insideThermometer[i])), TypeNames[TEMPERATURE], "", "", "");
      }
    } else {
      growBookPostValue("BAD_TEMP_VALUE_" + String(getAddressString(insideThermometer[i])), String(current_temp[i]));
    }
  }
  if (devices_count_good) {
    sum_tmp = sum_tmp / devices_count_good;
    temp_sum_value_prev = sum_tmp;
    growBookPostValue("temperature", String(sum_tmp));
  }
  return true;
}

TempAndHumidity read_dht(DHTesp &dhtSensor)
{
  TempAndHumidity ret = dhtSensor.getTempAndHumidity();
  humidity_value = ret.humidity;
  return ret;
}

/**
    Humidity sensor
*/
const bool sonsors_dht()
{
  return sonsor_dht(dhtMain);
}

const bool sonsor_dht(DHTesp &dhtSensor)
{
  TempAndHumidity ret = read_dht(dhtSensor);
  delay(20);
  float heat_index = dhtSensor.computeHeatIndex(ret.temperature, ret.humidity, false);
  //float dewPoint = dhtSensor.computeDewPoint(ret.temperature, ret.humidity, false);
  float absoluteHumidity = dhtSensor.computeAbsoluteHumidity(ret.temperature, ret.humidity, false);
  String serial_num = dhtSerialNumber(dhtSensor);
  if (String(heat_index) != "nan") {
    growBookPostEvent(String(ret.humidity), serial_num, TypeNames[HUMIDITY], String(absoluteHumidity), String(ret.temperature), String(heat_index));
  } else {
    if (String(dhtSensor.getStatusString()) != "TIMEOUT") {
      growBookPostValue("BAD_HUMIDITY_VALUE_" + serial_num, String(ret.humidity) + ":" + String(absoluteHumidity) + ":" + String(ret.temperature) + ":" + String(heat_index) + ":Status:" + dhtSensor.getStatusString());
    }
  }

  //  if (humidity < 10 || (humidity > 90 && humidity <= 100)) {
  //    growBookPostValue("humidity", String(humidity));
  //  }
  humidity_value_prev = humidity_value = ret.humidity;

  return true;
}

void growBookPostEvent(const String &value, const String &sensor, const String &type, const String &value1, const String &value2, const String &value3)
{
  String note = "";
  String url = "event/new?type=" + String(type);
  String postData;
  postData = String("rssi=") + urlencode(String(WiFi.RSSI())) +
             "&uptime=" + urlencode(String(uptime)) +
             "&value=" + urlencode(value) +
             "&sensor_id=" + urlencode(sensor) +
             "&note=" + urlencode(note) +
             "&plant_id=" + urlencode(getNodeName());
  if ( value1 != "" ) {
    postData += "&value1=" + urlencode(value1);
  }
  if ( value2 != "" ) {
    postData += "&value2=" + urlencode(value2);
  }
  if ( value3 != "" ) {
    postData += "&value3=" + urlencode(value3);
  }
  postData = (postData) + "&type=" + type;
  postTo(url, postData);
}

/**

*/
void growBookPostValue(const String &key, const String &value)
{
  postTo("plant/cli/" + urlencode(getNodeName()), String("key=") + urlencode(key) + "&value=" + urlencode(value));
}


String read_cmd(const String &postData)
{
  String ret = "";
  if ((CHECK_INTERNET_CONNECT && internet_access) || !CHECK_INTERNET_CONNECT) {
    String url = "plant/read_cmd/" + urlencode(getNodeName());
    String int_url = String(GROWBOOK_URL) + url;
    message("READ_CMD:" + int_url + " : \t" + String("postData:") + postData, DEBUG); // Print HTTP return code
#ifdef ESP8266
    ESP.wdtFeed();
#endif
    httpClient.begin(wifi_client, int_url);
    httpClient.setTimeout(2000);
    httpClient.addHeader("Content-Type", "application/x-www-form-urlencoded");  //Specify content-type header
    int httpCode = httpClient.POST(postData); // Send the request
    //  if ( httpCode == HTTP_CODE_OK) {
    if (httpCode < 0) {
      message(String(" !  -  Code:") + String(httpCode) + " " + String(" \t Message :") + httpClient.errorToString(httpCode) , DEBUG);
    }
    else {

      if (httpCode == HTTP_CODE_FOUND || httpCode == HTTP_CODE_OK) {
        ret = httpClient.getString(); // Get the response payload
        //message(String(" +  Code:") + String(httpCode) + " PayLoad:" + String(payload), INFO);    //Print request response payload
      } else {
        message(String(" -  Code:") + String(httpCode));// + " PayLoad:" + String(payload), DEBUG);    //Print request response payload
      }
    }
    httpClient.end();
  } else {
    message("No internet access", DEBUG);
    delay(20);
  }
  return ret;
}

const bool postTo(const String &url, const String &postData)
{
  bool ret = false;
  if ((CHECK_INTERNET_CONNECT && internet_access) || !CHECK_INTERNET_CONNECT) {
    String int_url = String(GROWBOOK_URL) + url;
    message("POST_TO:" + int_url + " : \t" + String("postData:") + postData, DEBUG); // Print HTTP return code
#ifdef ESP8266
    ESP.wdtFeed();
#endif
    httpClient.begin(wifi_client, int_url);
    httpClient.setTimeout(2000);
    httpClient.addHeader("Content-Type", "application/x-www-form-urlencoded");  //Specify content-type header
    short int httpCode = httpClient.POST(postData); // Send the request
    httpClient.end();
    if (httpCode < 0) {
      message(String(" !  -  Code:") + String(httpCode) + " " + String(" \t Message :") + httpClient.errorToString(httpCode) , DEBUG);
    } else {
      //String payload = httpClient.getString(); // Get the response payload
      if (httpCode == HTTP_CODE_FOUND || httpCode == HTTP_CODE_OK) {
        ret = true;
        //message(String(" +  Code:") + String(httpCode) + " PayLoad:" + String(payload), INFO);    //Print request response payload
      } else {
        message(String(" -  Code:") + String(httpCode));// + " PayLoad:" + String(payload), DEBUG);    //Print request response payload
      }
    }

  } else {
    message("No internet access", DEBUG);
    delay(20);
  }
  return ret;
}

const String getNodeName()
{
#ifdef ESP32
  return String(WiFi.getHostname()) + "_" + String(WiFi.macAddress());
#else
  return WiFi.hostname();
#endif
}
/**

*/
void growBookPostValues() {
  String url = "plant/cli/" + urlencode(getNodeName());
  String postData;
  postData = "uptime=" + urlencode(String(uptime)) + "&" + urlencode("light") + "=" + urlencode(String(light_enabled)) + "&version=" + urlencode(String(VERSION));
  if (humidity_value > 0) {
    postData += "&humidity=" + urlencode(String(humidity_value));
  }
  if (temp_sum_value_prev > 0) {
    postData += "&temperature=" + urlencode(String(temp_sum_value_prev));
  }
  if (temp_min_value_prev > 0) {
    postData += "&temperature_min=" + urlencode(String(temp_min_value_prev));
  }
  if (temp_max_value_prev > 0) {
    postData += "&temperature_max=" + urlencode(String(temp_max_value_prev));
  }
  if (hydro_value_prev > 0 || hydro_value_prev <= 100) {
    postData += "&hydrometer=" + urlencode(String(hydro_value_prev));
  }

#ifdef ESP8266

  postData += "&boot_mode=" + urlencode(String(ESP.getBootMode()));
  if (!reset_info_posted)
  {
    rst_info* rinfo = ESP.getResetInfoPtr();
    struct bootflags bflags = bootmode_detect();

    postData += "&reset_reason=" + urlencode(String(rinfo->reason));
    postData += "&reset_reason_str=" + urlencode(ESP.getResetReason().c_str());
    postData += "&reset_exccause=" + urlencode(String(rinfo->exccause));
    postData += "&reset_epc1=" + urlencode(String(rinfo->epc1));
    postData += "&reset_epc2=" + urlencode(String(rinfo->epc2));
    postData += "&reset_epc3=" + urlencode(String(rinfo->epc3));
    postData += "&reset_excvaddr=" + urlencode(String(rinfo->excvaddr));
    postData += "&reset_depc=" + urlencode(String(rinfo->depc));

    postData += "&bootflags_raw_rst_cause=" + urlencode(String(bflags.raw_rst_cause));
    postData += "&bootflags_raw_bootdevice=" + urlencode(String(bflags.raw_bootdevice));
    postData += "&bootflags_raw_bootmode=" + urlencode(String(bflags.raw_bootmode));
    postData += "&bootflags_rst_normal_boot=" + urlencode(String(bflags.rst_normal_boot));
    postData += "&bootflags_rst_reset_pin=" + urlencode(String(bflags.rst_reset_pin));
    postData += "&bootflags_rst_watchdog=" + urlencode(String(bflags.rst_watchdog));
    postData += "&bootflags_bootdevice_ram=" + urlencode(String(bflags.bootdevice_ram));
    postData += "&bootflags_bootdevice_flash=" + urlencode(String(bflags.bootdevice_flash));
  }

#endif

  postData += "&counter=" + urlencode(String(counter));
  postData += "&nodeName=" + urlencode(getNodeName());
  postData += "&SSID=" + urlencode(String(WiFi.SSID()));
  postData += "&BSSID=" + urlencode(String(WiFi.BSSIDstr()));
  postData += "&MAC_ADDRESS=" + urlencode(String(WiFi.macAddress()));
  postData += "&wifi_channel=" + urlencode(String(WiFi.channel())) + "&rssi=" + urlencode(String(WiFi.RSSI()));
  postData += "&flash_chip_mode=" + urlencode(String(ESP.getFlashChipMode()));
  postData += "&cpu_freq=" + urlencode(String(ESP.getCpuFreqMHz())) + "&" + urlencode("sdk_version") + "=" + urlencode(String(ESP.getSdkVersion()));

  bool res = postTo(url, postData);
  //  Check if we not posted reset info AND now we posted AND its PASS on transfer, we not want to update it again
  if (!reset_info_posted && res) {
    reset_info_posted = true;
  }

}

const String dhtSerialNumber(DHTesp &dhtSensor)
{
  return String("DHT") + String(dhtSensor.getModel()) + String(dhtSensor.getModel()) + "_-_" + getNodeName() + String("_-_") + String(dhtSensor.getPin());
}

DHTesp startSensor(DHTesp &dhtSensor, const unsigned int pin)
{
  message(" - - - - Starting DHT detection - - - - ");

  message("DHT start AM2302 mode...");
  dhtSensor.setup(pin, DHTesp::AM2302);   //
  delay(2000);
  TempAndHumidity _t = read_dht(dhtSensor);
  if (dhtSensor.getStatus() != 0) {
    message("DHT AM2302 failed", CRITICAL);
    message("DHT start DHT22 mode...");
    dhtSensor.setup(pin, DHTesp::DHT22);   //
    delay(2000);
    TempAndHumidity _t = read_dht(dhtSensor);
    message("_t value.humidity " + String(_t.humidity));
    if (_t.humidity == NAN || String(_t.humidity) == "nan" || _t.humidity < 5) {
      message("DHT [DHTesp::DHT22] failed. Value=" + String(_t.humidity));
      message("Setting [AUTO_DETECT]");
      dhtSensor.setup(pin, DHTesp::AUTO_DETECT);
      delay(2000);
      _t = read_dht(dhtSensor);
      if (_t.humidity == NAN || String(_t.humidity) == "nan" ) {
        message("DHT22 [DHTesp::DHT22] detect failed. Value:" + String(_t.humidity));
        message("Setting [DHTesp::DHT11]");
        dhtSensor.setup(pin, DHTesp::DHT11);
        delay(2000);
        _t = read_dht(dhtSensor);
        if (_t.humidity == NAN || String(_t.humidity) == "nan" ) {
          message("DHT11 [DHTesp::DHT11] detect failed. Value:" + String(_t.humidity));
          message("Setting [AUTO_DETECT]");
          dhtSensor.setup(pin, DHTesp::AUTO_DETECT);
          delay(2000);
          if (dhtSensor.getStatus() != 0) {
            message("DHT not defined", CRITICAL);
            dhtSensor.setup(-1, DHTesp::AUTO_DETECT);
          }
        } else {
          message("DHT11 success.", PASS);
        }
      } else {
        message("AUTO_DETECT success.", PASS);
      }
    } else {
      message("DHT DHT22 success.", PASS);
    }
  }

  message("DHT MODEL :" + dhtSerialNumber(dhtSensor), INFO);
  return dhtSensor;
}

/**

*/
const bool check_connectivity(const bool &force)
{
  // CEONNECTIVITY - CHECK PING
  if (CHECK_INTERNET_CONNECT || force) {
    if (counter % CHECK_INTERNET_CONNECTIVITY_CTR == 0 || !internet_access || force) {
      bool _ia = internet_access;
      internet_access = Ping.ping(pingServer, 2);
      if (!_ia) {
        message("Ping result is " + String(internet_access) + " avg_time_ms:" + String(Ping.averageTime()) + " Failures:" + String(internet_access_failures), INFO);
      }

      if (!internet_access) {
        internet_access_failures++;
        delay(500);
      } else {
        internet_access_failures = 0;
      }
    }

    if (internet_access_failures >= RECONNECT_AFTER_FAILS) {
      message("No Internet connection. internet_access_failures FLAG, reconnecting all.", CRITICAL);
      internet_access_failures = 0;
      reconnect_cnv();
    }
  } else {
    return true;
  }
  return internet_access;
}

/**

*/
void wifi_check()
{
  // WIFI CHECK
  if (WiFi.status() != WL_CONNECTED) {
    internet_access = 0;
    delay(2000);
#ifdef ESP8266
    ESP.wdtFeed();
#endif
    // Check if not connected , disable all network services, reconnect , enable all network services
    if (WiFi.status() != WL_CONNECTED) {
      internet_access = 0;
      message("WIFI DISCONNECTED", FAIL_t);
      reconnect_cnv();
      delay(1000);
    }
  }
}

/**
   Reconnect to wifi - in success enable all services and update time
*/
void reconnect_cnv()
{
  internet_access = 0;
  update_time_flag = true;
  delay(500);
  close_all_services();
  delay(1000);
  if (wifi_connect()) {
    timeClient.begin();
    //    server_start();
    delay(200);
    update_time();
  } else {
    message("Cannot reconnect to WIFI... ", FAIL_t);
    delay(1000);
#ifdef ESP8266
    ESP.wdtFeed();
#endif
  }
  update_time_flag = false;
}

/**
   Set WiFi connection and connect
*/
const bool wifi_connect()
{
  WiFi.mode(WIFI_STA);       //  Disable AP Mode - set mode to WIFI_AP, WIFI_STA, or WIFI_AP_STA.
  WiFi.begin(ssid, password);

  // Wait for connection
  message("Connecting to [" + String(ssid) + "][" + String(password) + "]...", INFO);
  int con_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
#ifdef ESP8266
    ESP.wdtFeed();
#endif
    Serial.print(".");
    con_counter++;
    if (con_counter % 20 == 0) {
      message("", INFO);
      message("Still connecting...", WARNING);
    }
    if (con_counter == 150) {
      message("", INFO);
      message("Cannot connect to [" + String(ssid) + "] ", FAIL_t);
      WiFi.disconnect();
      message(" ----> Disabling WiFi...", INFO);
      WiFi.mode(WIFI_OFF);
      return false;
    }
  }
  message("", INFO);
  message("Connected to [" + String(ssid) + "]  IP address: " + WiFi.localIP().toString(), PASS);
  //  Serial.print("PASS: );
  //  Serial.println(WiFi.localIP());
  if (MDNS.begin("esp8266")) {
    message("MDNS responder started", PASS);
  }
  message("-----------------------------------", INFO);
  return true;
}

void message(const String msg)
{
  message(msg, DEBUG);
}

/**
   Print message to Serial console
*/
void message(const String &msg, const enum LogType &lt)
{
  if (MESSAGE_OPT) {
    if (msg.length() == 0) {
      Serial.println(msg);
    } else {
      update_time_flag = true;
      String epoch = "";
      String f_time = "";
      if (internet_access) {
        epoch = String(timeClient.getEpochTime());
        f_time = String(timeClient.getFormattedTime());
      }
      update_time_flag = false;
      Serial.println(String(counter) + ":" + String(uptime) + ":" + epoch + " : " + f_time + " : " + String(stringFromLogType(lt)) + " : " + msg);
    }
  }
}

/**
  Close all network services
*/
void close_all_services()
{
  message(" ----> Starting close all network services <----", INFO);

  message(" ----> Closing NTP Client...", INFO);
  timeClient.end();
  //#ifdef ESP8266
  //  ESP.wdtFeed();
  //#endif
  //  message(" ----> Closing WEB Server...", INFO);
  //  server.close();
#ifdef ESP8266
  ESP.wdtFeed();
#endif
  message(" ----> Disconnecting WIFI...", INFO);
  WiFi.disconnect();
#ifdef ESP8266
  ESP.wdtFeed();
#endif
  message(" ----> Disabling WiFi...", INFO);
  WiFi.mode(WIFI_OFF);
#ifdef ESP8266
  ESP.wdtFeed();
#endif

  message(" ----> WiFi disabled...", INFO);

  //yield();
  message(" ----> Finished closing all network services <----", INFO);
}

/**

*/
void start_thermal()
{
  message(" - - - Start thermal - - - ", INFO);
  message("Found " + String(sensor.getDeviceCount()) + " Thermometer Dallas devices.", INFO);
  message("Parasite power is: " + String(sensor.isParasitePowerMode()), INFO);
  short int t_counter = 0;
  for (t_counter = 0; t_counter < sensor.getDeviceCount(); t_counter++) {
    if (!sensor.getAddress(insideThermometer[t_counter], t_counter)) {
      message("Unable to find address for Device " + String(t_counter) , CRITICAL);
    } else {
      // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensor.setResolution(insideThermometer[t_counter], TEMPERATURE_PRECISION);
      message("Device " + String(t_counter) + " [" + getAddressString(insideThermometer[t_counter]) + "] Resolution: " + String(sensor.getResolution(insideThermometer[t_counter])) , INFO);
    }
  }
}

/**
   Update time by NTP client
*/
void update_time()
{
  update_time_flag = true;
  noInterrupts();
  delay(100);
  if (timeClient.getEpochTime() < INCORRECT_EPOCH) {
    unsigned short counter_tmp = 0;
    while (timeClient.getEpochTime() < INCORRECT_EPOCH && counter_tmp < 10) {
      message("Incorrect time, trying to update: #:" + String(counter_tmp) + " \t " + String(timeClient.getEpochTime()) , CRITICAL);
      counter_tmp++;
      timeClient.update();
      timeClient.forceUpdate();
      timeClient.update();
      if (timeClient.getEpochTime() < INCORRECT_EPOCH) {
        delay(3000);
#ifdef ESP8266
        ESP.wdtFeed();
#endif
        //yield();
      } else {
        break;
      }
    }
  } else {
    timeClient.forceUpdate();
    timeClient.update();
  }
  interrupts();
  update_time_flag = false;
  message("Time updated." , PASS);
}

void print_reset_info()
{
#ifdef ESP8266
  rst_info* rinfo = ESP.getResetInfoPtr();

  Serial.printf("rinfo->reason:   %d, %s\n", rinfo->reason, ESP.getResetReason().c_str());
  Serial.printf("rinfo->exccause: %d\n", rinfo->exccause);
  Serial.printf("rinfo->epc1:     %d\n", rinfo->epc1);
  Serial.printf("rinfo->epc2:     %d\n", rinfo->epc2);
  Serial.printf("rinfo->epc3:     %d\n", rinfo->epc3);
  Serial.printf("rinfo->excvaddr: %d\n", rinfo->excvaddr);
  Serial.printf("rinfo->depc:     %d\n", rinfo->depc);

  struct bootflags bflags = bootmode_detect();

  Serial.printf("\nbootflags.raw_rst_cause: %d\n", bflags.raw_rst_cause);
  Serial.printf("bootflags.raw_bootdevice: %d\n", bflags.raw_bootdevice);
  Serial.printf("bootflags.raw_bootmode: %d\n", bflags.raw_bootmode);

  Serial.printf("bootflags.rst_normal_boot: %d\n", bflags.rst_normal_boot);
  Serial.printf("bootflags.rst_reset_pin: %d\n", bflags.rst_reset_pin);
  Serial.printf("bootflags.rst_watchdog: %d\n", bflags.rst_watchdog);

  Serial.printf("bootflags.bootdevice_ram: %d\n", bflags.bootdevice_ram);
  Serial.printf("bootflags.bootdevice_flash: %d\n", bflags.bootdevice_flash);

  if (bflags.raw_bootdevice == 1) {
    Serial.println("\n\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n!!!!!!!!! The sketch has just been uploaded over the serial link to the ESP8266");
    Serial.println("Beware: the device will freeze after it reboots in the following step.");
    Serial.println("It will be necessary to manually reset the device or to power cycle it");
    Serial.println("and thereafter the ESP8266 will continuously reboot.\n");
    //      Serial.println("\n\nRebooting with ESP.restart()");
    //      ESP.restart();
    delay(2000);
  } else {
    Serial.println("\n\n\n\n\n\n\n\n\n\n\nRESET_FOUND\n\n\n\n\n");
  }
#endif
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

/**
*/
void print_all_info()
{
  message("", DEBUG);
  message("FIRMWARE VERSION:" + String(VERSION) +
          "\t internet_access:" + String(internet_access) +
          "\t MIN_TEMPERATURE_TH \t:" + String(MIN_TEMPERATURE_TH) +
          "\t MIN_HUMIDITY_TH \t:" + String(MIN_HUMIDITY_TH), INFO);
  message("CHECK_INTERNET_CONNECT \t:" + String(CHECK_INTERNET_CONNECT), INFO);
  message("DHTesp_MODEL \t:" + String(dhtMain.getModel()), INFO);
  message("SdkVersion: " + String(ESP.getSdkVersion()) +
          "\t HostName:" + getNodeName() +
          " SSID:" + String(WiFi.SSID()) +
          " BSSID:" + WiFi.BSSIDstr() + "]" +
          " WiFiChannel:" + String(WiFi.channel()) + "[RSSI=" + WiFi.RSSI() + " MAC=" + WiFi.macAddress() + "]" +
          "\t FlashChip(Size/Speed/Mode):" + String(ESP.getFlashChipSize()) + "/" + String(ESP.getFlashChipSpeed()) + "/" + String(ESP.getFlashChipMode()) +
          " CpuFreqMHz: " + String(ESP.getCpuFreqMHz()) +
          " SketchSize: " + String(ESP.getSketchSize()) +
          " FreeSketchSpace: " + String(ESP.getFreeSketchSpace()), INFO);
#ifdef ESP8266
  message("ESP8266: BootMode:" + String(ESP.getBootMode())  +
          "\t BootVersion:" + ESP.getBootVersion() +
          "\t Flash Chip Id " + String(ESP.getFlashChipId()) +
          "\t CoreVersion: " + ESP.getCoreVersion(), INFO);
#endif
#ifdef ESP32
  message("ESP32: SketchMD5: " + String(ESP.getSketchMD5()) + "\t CycleCount: " + String(ESP.getCycleCount()) + "\t ChipRevision:" + String(ESP.getChipRevision()), INFO);
#endif
}
