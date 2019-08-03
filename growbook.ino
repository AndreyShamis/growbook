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
#include <ESP8266WiFi.h>
#include <ESP8266Ping.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "ESP8266httpUpdate.h"
#include "FS.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHTesp.h"
#include <ESP8266HTTPClient.h>
#include <Ticker.h>

/*******************************************************************************************************/
#define   VERSION                           0.3
// WiFi settings
#define   WIFI_SSID                         "RadiationG"
#define   WIFI_PASS                         "polkalol"

// NTP settings
#define   NTP_SERVER                        "1.asia.pool.ntp.org"   // Pool of ntp server http://www.pool.ntp.org/zone/asia
#define   NTP_TIME_OFFSET_SEC               10800                   // Time offset
#define   NTP_UPDATE_INTERVAL_MS            60000                   // NTP Update interval - 1 min

// Unchangeable settings
#define   INCORRECT_EPOCH                   200000                  // Minimum value for time epoch
#define   HIGH_TEMPERATURE                  125                      // If temperature bigger of this value we recheck it again
#define   LOW_TEMPERATURE                   -55
/*******************************************************************************************************/
// Thermometer, hydrometer, humidity light sensor and wire settings
#define   HYDROMETER_PIN                    A0
#define   LIGHT_SENSOR_D0                   D0
#define   HYDROMETER_D0_PIN                 D1
#define   ONE_WIRE_BUS                      D4                      // D4 2
#define   DHT_HUMIDITY_PIN                  D5                      // 14 D5 of NodeMCU is GPIO14

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

#define GROWBOOK_URL_NO_PORT                "192.168.1.206"
#define GROWBOOK_URL                        "http://192.168.1.206:8082/"

//#define GROWBOOK_URL                        "http://growbook.anshamis.com/"


// INTERRUPT
#define INTERRUPT_TIME                      120000000                //  600000      //12us
/*******************************************************************************************************/
enum LogType {
  INFO      = 0,
  WARNING   = 1,
  ERROR     = 2,
  PASS      = 3,
  FAIL_t    = 4,
  CRITICAL  = 5,
  DEBUG     = 6,
} ;

enum SensorType {
  HUMIDITY = 0,
  TEMPERATURE = 1,
  HYDROMETER = 2,
  LIGHT = 3,
};


struct bootflags
{
  unsigned char raw_rst_cause : 4;
  unsigned char raw_bootdevice : 4;
  unsigned char raw_bootmode : 4;
  unsigned char rst_normal_boot : 1;
  unsigned char rst_reset_pin : 1;
  unsigned char rst_watchdog : 1;
  unsigned char bootdevice_ram : 1;
  unsigned char bootdevice_flash : 1;
};

struct bootflags bootmode_detect(void) {
  int reset_reason, bootmode;
  asm (
    "movi %0, 0x60000600\n\t"
    "movi %1, 0x60000200\n\t"
    "l32i %0, %0, 0x114\n\t"
    "l32i %1, %1, 0x118\n\t"
    : "+r" (reset_reason), "+r" (bootmode) /* Outputs */
    : /* Inputs (none) */
    : "memory" /* Clobbered */
  );

  struct bootflags flags;
  flags.raw_rst_cause = (reset_reason & 0xF);
  flags.raw_bootdevice = ((bootmode >> 0x10) & 0x7);
  flags.raw_bootmode = ((bootmode >> 0x1D) & 0x7);
  flags.rst_normal_boot = flags.raw_rst_cause == 0x1;
  flags.rst_reset_pin = flags.raw_rst_cause == 0x2;
  flags.rst_watchdog = flags.raw_rst_cause == 0x4;
  flags.bootdevice_ram = flags.raw_bootdevice == 0x1;
  flags.bootdevice_flash = flags.raw_bootdevice == 0x3;

  return flags;
}

/*******************************************************************************************************/

String              TypeNames[4] = {
  "App%5CEntity%5CEvents%5CEventHumidity",
  "App%5CEntity%5CEvents%5CEventTemperature",
  "App%5CEntity%5CEvents%5CEventSoilHydrometer",
  "App%5CEntity%5CEvents%5CEventLight"
};
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
OneWire             oneWire(ONE_WIRE_BUS);
DallasTemperature   sensor(&oneWire);
ESP8266WebServer    server(80);
DeviceAddress       insideThermometer[NUMBER_OF_SENSORS];       // arrays to hold device address
WiFiUDP             ntpUDP;
IPAddress           pingServer (8, 8, 8, 8);    // Ping server
float               current_temp[NUMBER_OF_SENSORS];
/**
    You can specify the time server pool and the offset (in seconds, can be changed later with setTimeOffset()).
    Additionaly you can specify the update interval (in milliseconds, can be changed using setUpdateInterval()). */
NTPClient           timeClient(ntpUDP, NTP_SERVER, NTP_TIME_OFFSET_SEC, NTP_UPDATE_INTERVAL_MS);
HTTPClient          httpClient;    //Declare object of class HTTPClient
DHTesp              dhtMain;
float               temp_sum_value_prev = 0;
float               temp_min_value_prev = 0;
float               temp_max_value_prev = 0;
float               hydro_value_prev = -1;
float               humidity_value_prev = 0;
float               humidity_value = 0;
bool                light_enabled = false;
volatile unsigned long       boot_time = 0;
volatile unsigned long       uptime = 0;
WiFiClient          wifi_client;

volatile bool       update_time_flag = false;
//volatile unsigned int        interruptCounter = 0;
/*******************************************************************************************************/

//ADC_MODE(ADC_VCC);              // This disable ADC read!
float   getTemperature(const int dev = 0);
/**
 ****************************************************************************************************
 ****************************************************************************************************
*/


//=======================================================================
void ICACHE_RAM_ATTR onTimerISR()
{
  //if (interruptCounter % 4 == 0) {
  if (boot_time > 0 && !update_time_flag) {
    uptime = timeClient.getEpochTime() - boot_time;
  }
  //}
  //message("Tick");
  //interruptCounter++;
  //  if (interruptCounter > 30000) {
  //    interruptCounter = 0;
  //  }
  timer1_write(INTERRUPT_TIME);//12us
}

/**
  Setup the controller
*/
void setup(void) {
  //pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(LIGHT_SENSOR_D0, INPUT);
  if (CHECK_INTERNET_CONNECT) {
    internet_access = 0;
  }
  else {
    internet_access = 1;
  }
  //disableLoad();
  sensor.begin();
  Serial.begin(UART_BAUD_RATE);
  Serial.println("");
  message("Serial communication started.", PASS);

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

  message("Starting SPIFFS....", INFO);
  SPIFFS.begin();
  message("SPIFFS startted.", PASS);
  //message("Compile SPIFFS", INFO);
  //  SPIFFS.format();



  //Serial.println(build_index());
  wifi_connect();
  server_start();
  timeClient.begin();
  start_thermal();
  //timeClient.update();
  delay(1000);
  timeClient.forceUpdate();
  delay(1000);
  dhtMain = startSensor(dhtMain, DHT_HUMIDITY_PIN);
  message(" ----> All started <----", PASS);
  delay(1000);
  wifi_check();
  print_all_info();
  ESP.wdtEnable(10000);
  ESP.wdtDisable();
  delay(1000);
  check_connectivity(true);
  while (counter < 100) {
    if ((CHECK_INTERNET_CONNECT && internet_access) || !CHECK_INTERNET_CONNECT) {
      message(" ----> internet_access is OK<----", PASS);
      break;
    }
    ESP.wdtFeed();
    delay(100);
    counter += 1;
    if (counter == 50) {
      check_connectivity(true);
    }
    message("internet_access: " + String(internet_access) + " CHECK_INTERNET_CONNECT:" + String(CHECK_INTERNET_CONNECT), PASS);
  }
  counter = 0;
  update_time();
  message(" ----> Start Loop <----", PASS);
  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(INTERRUPT_TIME); //120000 us
}


/**
  /////////////////////// L O O P   F U N C T I O N //////////////////////////
  ////////////////////////////////////////////////////////////////////////////
*/

void loop(void) {
  // WEB SERVER
  if (internet_access && (boot_time == 0 || boot_time < INCORRECT_EPOCH)) {
    boot_time = timeClient.getEpochTime();
    message("Boot time updated...", DEBUG);
  }

  server.handleClient();
  ESP.wdtFeed();
  wifi_check();

  check_light(false);
  if (counter % CHECK_HUMIDITY_COUNTER == 0) {
    read_dht(dhtMain);

  }
  // SENSORS  ---------------------------------------------------------------------------------------
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
  }
  if (counter % CHECK_SENSORS_8 == 0) {
    sonsors_dht();
    check_light(false);
  }

  if (counter % CALL_SERVER_COUNTER == 0 && internet_access) {
    growBookPostValues();
    ESP.wdtFeed();
  }

  check_connectivity(false);
  if (counter == 20) {
    print_all_info();

  }

  if (counter % NTP_UPDATE_COUNTER == 0) {
    if (internet_access) {
      message("Starting update the time...", DEBUG);
      update_time();
    }
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

void read_cmd_flow()
{
  String output = read_cmd("");
  if (output.length() > 200) {
    return;
  }
  String key = getValue(output, ':', 0);
  String value = getValue(output, ':', 1);
  //String key, value = "";
  //char buff[100];
  message("here Received " + output);
  // output.toCharArray(buff, output.length() + 1);
  //  message("scan buff LEN IS  " + String(output.length()));
  // sscanf(buff, "%s ^ %s ^^", &key, &value);\
  // key.trim();
  //  value.trim();
  message("check KEY len " + String(key.length()));
  if (key.length() > 1)
  {
    message("Print msg");
    message("READ cmd:" + output + " KEY=" + String(key) + " VALUE=" + String(value));
    if (key == "reboot" && value.toInt() == 1) {
      message("Resetting ESP" , WARNING);
      delay(500);
      ESP.restart();
    }
    if (key == "firmwareUpdate") {
      String url = "/1.bin"; // + String(value);
      delay(500);
      message("Update firmware from " + String(GROWBOOK_URL_NO_PORT) + url , WARNING);

      delay(500);
      ESP.wdtFeed();
      ESP.wdtEnable(30000);
      ESP.wdtDisable();

      ESP.wdtFeed();
      ESPhttpUpdate.rebootOnUpdate(true);
      ESPhttpUpdate.followRedirects(true);
      message("Start");
      HTTPUpdateResult ret = ESPhttpUpdate.update(GROWBOOK_URL_NO_PORT , 8082, url);
      message("END");
      //HTTPUpdateResult ret = ESPhttpUpdate.update("http://192.168.1.206:8082/firmware/growbook_v0.4.ino.bin");
      ESP.wdtEnable(5000);
      ESP.wdtFeed();
      //HTTPUpdateResult ret = ESPhttpUpdate.update(wifi_client, "http://192.168.1.206:8082/firmware/growbook_v0.4.ino.bin");
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
    }
  }
}


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

bool sensors_hydrometer() {
  float hydro_value = analogRead(HYDROMETER_PIN);
  float hydro_value_src = hydro_value;
  hydro_value = map(hydro_value, 0, 1024, 1000, 0) / 10.0;
  //sensorsSingleLog += " HYDRO:[" + String(hydro_value) + " :src=" + hydro_value_src + "]";
  String model = "HYDRO_" + String(HYDROMETER_PIN) + "_";
  bool epoch_trigger = timeClient.getEpochTime() % 7 == 0;
  if (hydro_value >= 0 && hydro_value <= 100) {
    if (hydro_value != hydro_value_prev || hydro_value_prev == 0 || epoch_trigger) {
      growBookPostEvent(String(hydro_value), model + "_-_" + String(WiFi.hostname()) + String("_-_0"), TypeNames[HYDROMETER], "", "", "");
      hydro_value_prev = hydro_value;
    }
  } else {
    message("Hydrometer bad value: " + String(hydro_value) + " -- SRC:" + String(hydro_value_src), CRITICAL);
  }
  growBookPostValue("hydrometer", String(hydro_value));

  return true;
}

bool check_light(bool verbose) {
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
bool sensors_light() {

  check_light(true);
  //sensorsSingleLog += " LIGHT IS " + String(light_enabled) + " ";
  //if (change_found) {
  String serail = "LDR_" + String(WiFi.hostname()) + String("_" + String(LIGHT_SENSOR_D0));
  growBookPostValue("light", String(light_enabled));
  //growBookPostEvent(String(light_enabled), serail, TypeNames[LIGHT], "", "", "");
  /// }
  return true;
}

bool sonsors_dallas() {
  //sensorsSingleLog = "Temperature:";
  int devices_count = sensor.getDeviceCount();
  float sum_tmp = 0;
  temp_max_value_prev = LOW_TEMPERATURE;
  temp_min_value_prev = HIGH_TEMPERATURE;
  for (int i = 0; i < devices_count; i++) {
    float prevTmp = current_temp[i];
    float tmp_1 = 0;
    float tmp_2 = 1;
    current_temp[i] = getTemperature(i);
    if (prevTmp != current_temp[i]) {
      short int _c = 0;
      while (tmp_1 != tmp_2 && _c < 10) {
        _c++;
        delay(20 * _c);
        tmp_1 = getTemperature(i);
        tmp_2 = getTemperature(i);
        if (tmp_1 > HIGH_TEMPERATURE && tmp_1 == tmp_2) {
          message("High temperature found, recheck. Current" + String(tmp_1) + " Threshhold[HIGH_TEMPERATURE]: " + String(HIGH_TEMPERATURE), CRITICAL);
          tmp_1 = 0;
          delay(200);
        }
      }
      current_temp[i] = tmp_1;
    }

    float tmp_diff = prevTmp - current_temp[i];
    if (current_temp[i] > LOW_TEMPERATURE && current_temp[i] < HIGH_TEMPERATURE) {
      sum_tmp += current_temp[i];
      temp_max_value_prev = max(temp_max_value_prev, current_temp[i]);
      temp_min_value_prev = min(temp_min_value_prev, current_temp[i]);
      // sensorsSingleLog += " \t " + String(i) + ": " + String(current_temp[i]) + " C \t | ";
      bool epoch_trigger = timeClient.getEpochTime() % 7 == 0;
      if (fabs(tmp_diff) > MIN_TEMPERATURE_TH || epoch_trigger) {
        if (epoch_trigger) {
          message("- ---          ****************************** ---------- EPOCH TRIGGER");
        }
        growBookPostEvent(String(current_temp[i]), String(getAddressString(insideThermometer[i])), TypeNames[TEMPERATURE], "", "", "");
      }
    } else {
      growBookPostValue("BAD_TEMERATURE_VALUE_" + String(getAddressString(insideThermometer[i])), String(current_temp[i]));
    }
  }
  if (devices_count) {
    sum_tmp = sum_tmp / devices_count;
    temp_sum_value_prev = sum_tmp;
    growBookPostValue("temperature", String(sum_tmp));
  }
  return true;
}

TempAndHumidity read_dht(DHTesp &dhtSensor) {
  TempAndHumidity ret = dhtSensor.getTempAndHumidity();
  humidity_value = ret.humidity;
  return ret;
}

/**
    Humidity sensor
*/
bool sonsors_dht() {
  return sonsor_dht(dhtMain);
}

bool sonsor_dht(DHTesp &dhtSensor) {
  TempAndHumidity ret = read_dht(dhtSensor);
  delay(20);
  float heat_index = dhtSensor.computeHeatIndex(ret.temperature, ret.humidity, false);
  float dewPoint = dhtSensor.computeDewPoint(ret.temperature, ret.humidity, false);
  float absoluteHumidity = dhtSensor.computeAbsoluteHumidity(ret.temperature, ret.humidity, false);
  bool epoch_trigger = timeClient.getEpochTime() % 17 == 0;
  //sensorsSingleLog += String(" \t Humidity:") + "DHT Status [" + dhtSensor.getStatusString() + "]\tHumidity: [" + ret.humidity + "%] \t TMP:" + ret.temperature + "C - Heat Index: [" + heat_index + " C]" + " DewPoint : " + String(dewPoint) + " absoluteHumidity:" + String(absoluteHumidity) + " dec size:" + String(dhtSensor.getNumberOfDecimalsHumidity());
  if (fabs(humidity_value_prev - ret.humidity)  > MIN_HUMIDITY_TH || epoch_trigger) {
    if (epoch_trigger) {
      message("- ---          ****************************** ---------- EPOCH TRIGGER ----------------------------- DHT");
    }
    String model = String("DHT") + String(dhtSensor.getModel()) + String(dhtSensor.getModel());
    if (String(heat_index) != "nan") {
      growBookPostEvent(String(ret.humidity), model + "_-_" + String(WiFi.hostname()) + String("_-_") + String(dhtSensor.getPin()), TypeNames[HUMIDITY], String(absoluteHumidity), String(ret.temperature), String(heat_index));
    }
  }
  //  if (humidity < 10 || (humidity > 90 && humidity <= 100)) {
  //    growBookPostValue("humidity", String(humidity));
  //  }
  humidity_value_prev = humidity_value = ret.humidity;

  return true;
}

void growBookPostEvent(String value, String sensor, String type, String value1, String value2, String value3) {
  String note = "";
  String url = "event/new?type=" + String(type);
  //    httpClient.setTimeout(5000);
  String postData;
  postData = String("rssi=") + urlencode(String(WiFi.RSSI())) + "&version=" + urlencode(String(VERSION)) + "&uptime=" + urlencode(String(uptime)) + "&value=" + urlencode(value) + "&sensor_id=" + urlencode(sensor) + "&note=" + urlencode(note) + "&plant_id=" + urlencode(WiFi.hostname());
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
void growBookPostValue(String key, String value) {
  String url = "plant/cli/" + urlencode(WiFi.hostname());// + "/" + urlencode(key) + "/" + urlencode(value);
  //    httpClient.setTimeout(2000);
  String postData;
  postData = String("key=") + urlencode(key) + "&value=" + urlencode(value);
  postTo(url, postData);
}

String read_cmd(const String &postData) {
  String ret = "";
  if ((CHECK_INTERNET_CONNECT && internet_access) || !CHECK_INTERNET_CONNECT) {
    String url = "plant/read_cmd/" + urlencode(WiFi.hostname());
    String int_url = String(GROWBOOK_URL) + url;
    message(int_url + " : \t" + String("postData:") + postData, DEBUG); // Print HTTP return code
    httpClient.begin(wifi_client, int_url);
    httpClient.setTimeout(4000);
    httpClient.addHeader("Content-Type", "application/x-www-form-urlencoded");  //Specify content-type header
    ESP.wdtDisable();
    ESP.wdtFeed();
    int httpCode = httpClient.POST(postData); // Send the request
    ESP.wdtFeed();
    ESP.wdtEnable(5000);
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

void postTo(const String &url, const String &postData) {
  if ((CHECK_INTERNET_CONNECT && internet_access) || !CHECK_INTERNET_CONNECT) {
    String int_url = String(GROWBOOK_URL) + url;
    httpClient.begin(wifi_client, int_url);
    httpClient.setTimeout(2000);
    httpClient.addHeader("Content-Type", "application/x-www-form-urlencoded");  //Specify content-type header
    message(int_url + " : \t" + String("postData:") + postData, DEBUG); // Print HTTP return code
    ESP.wdtDisable();
    ESP.wdtFeed();
    int httpCode = httpClient.POST(postData); // Send the request
    ESP.wdtFeed();
    ESP.wdtEnable(5000);
    //  if ( httpCode == HTTP_CODE_OK) {
    if (httpCode < 0) {
      message(String(" !  -  Code:") + String(httpCode) + " " + String(" \t Message :") + httpClient.errorToString(httpCode) , DEBUG);
    }
    else {
      //String payload = httpClient.getString(); // Get the response payload
      if (httpCode == HTTP_CODE_FOUND || httpCode == HTTP_CODE_OK) {
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
}
/**

*/
void growBookPostValues() {
  String url = "plant/cli/" + urlencode(WiFi.hostname());
  //    httpClient.setTimeout(2000);
  String postData;
  postData = "uptime=" + urlencode(String(uptime)) + "&" + urlencode("light_enabled") + "=" + urlencode(String(light_enabled));
  if (humidity_value > 0) {
    postData += "&humidity=" + urlencode(String(humidity_value));
  }
  if (temp_sum_value_prev > 0) {
    postData += "&temerature=" + urlencode(String(temp_sum_value_prev));
  }
  if (temp_min_value_prev > 0) {
    postData += "&temerature_min=" + urlencode(String(temp_min_value_prev));
  }
  if (temp_max_value_prev > 0) {
    postData += "&temerature_max=" + urlencode(String(temp_max_value_prev));
  }
  if (hydro_value_prev > 0 || hydro_value_prev <= 100) {
    postData += "&hydrometer=" + urlencode(String(hydro_value_prev));
  }

  postData += "&wifi_channel=" + urlencode(String(WiFi.channel())) + "&" + urlencode("rssi") + "=" + urlencode(String(WiFi.RSSI()));
  postData += "&flash_chip_mode=" + urlencode(String(ESP.getFlashChipMode())) + "&" + urlencode("boot_mode") + "=" + urlencode(String(ESP.getBootMode()));
  postData += "&cpu_freq=" + urlencode(String(ESP.getCpuFreqMHz())) + "&" + urlencode("sdk_version") + "=" + urlencode(String(ESP.getSdkVersion()));

  postTo(url, postData);
}


DHTesp startSensor(DHTesp &dhtSensor, const unsigned int pin)
{
  message("DHT start DHT22 mode...");
  dhtSensor.setup(pin, DHTesp::DHT22);   //
  delay(1000);
  TempAndHumidity _t = read_dht(dhtSensor);
  message("_t value.humidity " + String(_t.humidity));
  if (_t.humidity == NAN || String(_t.humidity) == "nan" || _t.humidity < 5) {
    message("DHT auto detect failed. Seting AUTO_DETECT. Value " + String(_t.humidity));
    dhtSensor.setup(pin, DHTesp::AUTO_DETECT);
    _t = read_dht(dhtSensor);
    if (_t.humidity == NAN || String(_t.humidity) == "nan" ) {
      message("DHT22 detect failed. Seting DHT11. Value " + String(_t.humidity));
      dhtSensor.setup(pin, DHTesp::DHT11);
      delay(2000);
      _t = read_dht(dhtSensor);
      if (_t.humidity == NAN || String(_t.humidity) == "nan" ) {
        message("DHT11 detect failed. Seting AUTO_DETECT. Value " + String(_t.humidity));
        dhtSensor.setup(pin, DHTesp::AUTO_DETECT);
      } else {
        message("DHT11 success.", PASS);
      }
    } else {
      message("DHT22 success.", PASS);
    }
  } else {
    message("DHT AUTO_DETECT success.", PASS);
  }
  String dht_model = "DHT" + String(dhtSensor.getModel()) + String(dhtSensor.getModel());
  message("DHT MODEL :" + dht_model, INFO);
  return dhtSensor;
}

/**

*/
bool check_connectivity(bool force)
{
  // CEONNECTIVITY - CHECK PING
  if (CHECK_INTERNET_CONNECT || force) {
    if (counter % CHECK_INTERNET_CONNECTIVITY_CTR == 0 || !internet_access || force) {
      bool _ia = internet_access;
      internet_access = Ping.ping(pingServer, 2);
      if (!_ia) {
        message("Ping result is " + String(internet_access) + " avg_time_ms:" + String(Ping.averageTime()), INFO);
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
    ESP.wdtFeed();
    // Check if not connected , disable all network services, reconnect , enable all network services
    if (WiFi.status() != WL_CONNECTED) {
      message("WIFI DISCONNECTED", FAIL_t);
      reconnect_cnv();
      delay(1000);
    }
  }
}

/**
   Reconnect to wifi - in success enable all services and update time
*/
void reconnect_cnv() {
  close_all_services();
  delay(1000);
  if (wifi_connect()) {
    timeClient.begin();
    server_start();
    delay(200);
    update_time();
  } else {
    message("Cannot reconnect to WIFI... ", FAIL_t);
    delay(1000);
    ESP.wdtFeed();
  }
}

/**
   Set WiFi connection and connect
*/
bool wifi_connect() {
  WiFi.mode(WIFI_STA);       //  Disable AP Mode - set mode to WIFI_AP, WIFI_STA, or WIFI_AP_STA.
  WiFi.begin(ssid, password);

  // Wait for connection
  message("Connecting to [" + String(ssid) + "][" + String(password) + "]...", INFO);
  int con_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    ESP.wdtFeed();
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

/**
  Keep type of mesages
*/
//static inline char *stringFromLogType(enum LogType lt)
static const char *stringFromLogType(const enum LogType lt) {
  static const char *strings[] = {"INFO", "WARN", "ERROR", "PASS", "FAIL", "CRITICAL", "DEBUG"};
  return strings[lt];
}

void message(const String msg) {
  message(msg, DEBUG);
}

/**
   Print message to Serial console
*/
void message(const String msg, const enum LogType lt) {
  if (MESSAGE_OPT) {
    if (msg.length() == 0) {
      Serial.println(msg);
    } else {
      Serial.println(String(uptime) + ":" + String(timeClient.getEpochTime()) + " : " + timeClient.getFormattedTime() + " : " + String(stringFromLogType(lt)) + " : " + msg);
    }
  }
}


/**
   Start WEB server
*/
void server_start() {
  server.on("/", handleRoot);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  //  server.on("/el", []() {
  //    enableLoad();
  //    loadMode = MANUAL;
  //    handleRoot();
  //  });
  //  server.on("/dl", []() {
  //    disableLoad();
  //    loadMode = MANUAL;
  //    handleRoot();
  //  });
  //  server.on("/setDallasIndex", []() {
  //    uploadAndSaveOutsideThermometerIndex();
  //    handleRoot();
  //  });
  //  server.on("/keep", []() {
  //    last_disable_epoch = 0;
  //    saveLoadMode();
  //    handleRoot();
  //  });

  server.onNotFound(handleNotFound);
  message("Staring HTTP server...", INFO);
  server.begin();
  message("HTTP server started", PASS);
}


/**
  Close all network services
*/
void close_all_services() {
  message(" ----> Starting close all network services <----", INFO);

  message(" ----> Closing NTP Client...", INFO);
  timeClient.end();
  ESP.wdtFeed();
  message(" ----> Closing WEB Server...", INFO);
  server.close();
  ESP.wdtFeed();
  message(" ----> Disconnecting WIFI...", INFO);
  WiFi.disconnect();
  ESP.wdtFeed();
  message(" ----> Disabling WiFi...", INFO);
  WiFi.mode(WIFI_OFF);
  ESP.wdtFeed();
  message(" ----> WiFi disabled...", INFO);

  yield();
  message(" ----> Finished closing all network services <----", INFO);
}

/**

*/
void start_thermal() {
  message("Found " + String(sensor.getDeviceCount()) + " Thermometer Dallas devices.", INFO);
  message("Parasite power is: " + String(sensor.isParasitePowerMode()), INFO);

  for (int i = 0; i < sensor.getDeviceCount(); i++) {
    if (!sensor.getAddress(insideThermometer[i], i)) {
      message("Unable to find address for Device " + String(i) , CRITICAL);
    }
    else {
      // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensor.setResolution(insideThermometer[i], TEMPERATURE_PRECISION);
      message("Device " + String(i) + " [" + getAddressString(insideThermometer[i]) + "] Resolution: " + String(sensor.getResolution(insideThermometer[i])) , INFO);
    }
  }

}

/**
*/
String build_index() {

  String ret_js = String("") + "load = \n{" +
                  "'internet_access': '" + String(internet_access) + "'," +
                  "'current_temperature_0': '" + String(current_temp[0]) + "'," +
                  "'current_temperature_1': '" + String(current_temp[1]) + "'," +
                  "'current_temperature_2': '" + String(current_temp[2]) + "'," +
                  "'current_temperature_3': '" + String(current_temp[3]) + "'," +
                  "'flash_chip_id': '" + String(ESP.getFlashChipId()) + "'," +
                  "'flash_chip_size': '" + String(ESP.getFlashChipSize()) + "'," +
                  "'flash_chip_speed': '" + String(ESP.getFlashChipSpeed()) + "'," +
                  "'flash_chip_mode': '" + String(ESP.getFlashChipMode()) + "'," +
                  "'core_version': '" + ESP.getCoreVersion() + "'," +
                  "'sdk_version': '" + String(ESP.getSdkVersion()) + "'," +
                  "'boot_version': '" + ESP.getBootVersion() + "'," +
                  "'boot_mode': '" + String(ESP.getBootMode()) + "'," +
                  "'cpu_freq': '" + String(ESP.getCpuFreqMHz()) + "'," +
                  "'mac_addr': '" + WiFi.macAddress() + "'," +
                  "'wifi_channel': '" + String(WiFi.channel()) + "'," +
                  "'rssi': '" + WiFi.RSSI() + "'," +
                  "'sketch_size': '" + String(ESP.getSketchSize()) + "'," +
                  "'free_sketch_size': '" + String(ESP.getFreeSketchSpace()) + "'," +
                  "'temperature_precision': '" + String(TEMPERATURE_PRECISION) + "'," +
                  "'time_str': '" + timeClient.getFormattedTime() + "'," +
                  "'time_epoch': '" + timeClient.getEpochTime() + "'," +
                  "'hostname': '" + WiFi.hostname() + "'" +
                  "};\n";
  String ret = String("") + "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><title>Load Info</title></head>" +
               " <script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.0/jquery.min.js'></script>\n" +
               " <script src='http://tm.anshamis.com/js/heater.js'></script>\n" +
               " <link rel='stylesheet' type='text/css' href='http://tm.anshamis.com/css/heater.css'>\n" +
               "<body><script>" + ret_js + "</script>\n" +
               "<div id='content'></div>" +
               "<script>\n " +               "$(document).ready(function(){ onLoadPageLoad(); });</script>\n" +
               "</body></html>";
  return ret;
}

/**
   Update time by NTP client
*/
void update_time() {
  update_time_flag = true;
  if (timeClient.getEpochTime() < INCORRECT_EPOCH) {
    unsigned short counter_tmp = 0;
    while (timeClient.getEpochTime() < INCORRECT_EPOCH && counter_tmp < 10) {
      message("Incorrect time, trying to update: #:" + String(counter_tmp) , CRITICAL);
      counter_tmp++;
      timeClient.update();
      timeClient.forceUpdate();
      timeClient.update();
      if (timeClient.getEpochTime() < INCORRECT_EPOCH) {
        delay(1000 + 500 * counter_tmp);
        yield();
      }
      else {
        break;
      }
    }
  }
  else {
    timeClient.forceUpdate();
    timeClient.update();
  }
  update_time_flag = false;
  message("Time updated." , PASS);
}

/**
   Get Temperature
*/
float getTemperature(const int dev/*=0*/) {
  //message("Requesting device " + String(dev), DEBUG);
  sensor.setWaitForConversion(false);   // makes it async
  sensor.requestTemperatures();
  sensor.setWaitForConversion(true);    // makes it async
  return sensor.getTempCByIndex(dev);
  //return sensor.getTempC(insideThermometer[dev]);
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
///**
//   Enable Load
//*/
//void enableLoad() {
//  float current_temp_tmp = getTemperature(outsideThermometerIndex);
//  if (current_temp_tmp > MAX_POSSIBLE_TMP) {
//    message("Current temperature is bigger of possible maximum. " + String(current_temp_tmp) + ">" + String(MAX_POSSIBLE_TMP), ERROR);
//  }
//  else {
//    secure_disabled = false;
//    heaterStatus = 1;
//    digitalWrite(LOAD_VCC, 1);
//  }
//}

///**
//   Disable Load
//*/
//void disableLoad() {
//  heaterStatus = 0;
//  last_disable_epoch = timeClient.getEpochTime();
//  digitalWrite(LOAD_VCC, 0);
//}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

///***
//  WEB Server function
//*/
//void uploadAndSaveOutsideThermometerIndex() {
//  for (uint8_t i = 0; i < server.args(); i++) {
//    if (server.argName(i) == "outTmpIndex") {
//      saveOutsideThermometerIndex(server.arg(i).toInt());
//      message("saveOutsideThermometerIndex " + String(server.arg(i)), INFO);
//    }
//  }
//}

///***
//  WEB Server function
//*/
//void saveLoadMode() {
//  for (uint8_t i = 0; i < server.args(); i++) {
//    if (server.argName(i) == "temperatureKeep") {
//      temperatureKeep = server.arg(i).toFloat();
//      loadMode = KEEP;
//      message("Keep temperature ", INFO);
//      if (temperatureKeep > MAX_POSSIBLE_TMP) {
//        temperatureKeep = MAX_POSSIBLE_TMP;
//        message("Override Keep temperature to MAX_POSSIBLE_TMP " + String(temperatureKeep), INFO);
//      }
//    }
//  }
//}

/**
  WEB Server function
*/
void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: " + server.uri() + "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args() + "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

/**
  WEB Server function
*/
void handleRoot() {

  //  for (uint8_t i=0; i<server.args(); i++){
  //    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  //  }
  //  message += server.client();
  String message = build_index();
  server.send(200, "text/html", message);
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////



/**
   print temperature to serial
*/
void printTemperatureToSerial() {
  int dc = sensor.getDeviceCount();
  for (int i = 0 ; i < dc; i++) {
    message("Temperature[" + String(i) + "] C: " + String(getTemperature(i)), INFO);
    //Serial.println("INFO: Temperature[" + String(i) + "] C: " + String(getTemperature(i)));
  }
}

/**

*/
String get_thermometers_addr() {
  String data = "[";
  int i = 0;
  int dev_counter = sensor.getDeviceCount();
  for (i = 0; i < dev_counter; i++) {
    data = data + String("\"") + String(getAddressString(insideThermometer[i])) + String("\" , ");
  }
  data = data + "]";
  return data;
}

/**
  Convert Dallas Address to String
*/
String getAddressString(const DeviceAddress deviceAddress) {
  String ret = "";
  uint8_t i;
  for (i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) {
      ret += "0";
    }
    ret += String(deviceAddress[i], HEX);
    if (i < 7) {
      ret += ":";
    }
  }
  return ret;
}

/**

*/
String urlencode(const String &s) {
  static const char lookup[] = "0123456789abcdef";
  String result;
  size_t len = s.length();

  for (size_t i = 0; i < len; i++) {
    const char c = s[i];
    if (('0' <= c && c <= '9') || ('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || (c == '-' || c == '_' || c == '.' || c == '~')) {
      result += c;
    } else {
      result += "%" + String(lookup[(c & 0xf0) >> 4]) + String(lookup[c & 0x0f]);
    }
  }
  return result;
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

/**
  Write to file content on SPIFFS
*/
void save_setting(const char* fname, String value) {
  File f = SPIFFS.open(fname, "w");
  if (!f) {
    Serial.print("Cannot open file:");
    Serial.println(fname);
    return;
  }
  f.println(value);
  Serial.print("Written:");
  Serial.println(value);
  f.close();
}

/**
  Read file content from SPIFFS
*/
String read_setting(const char* fname) {
  String s      = "";
  File f = SPIFFS.open(fname , "r");
  if (!f) {
    Serial.print("file open failed:");
    Serial.println(fname);
  }
  else {
    s = f.readStringUntil('\n');
    f.close();
  }
  return s;
}

/**
 ****************************************************************************************************
*/

/**

*/
void print_all_info() {
  message("", DEBUG);
  message("internet_access \t:" + String(internet_access), INFO);
  message("MIN_TEMPERATURE_TH \t:" + String(MIN_TEMPERATURE_TH), INFO);
  message("MIN_HUMIDITY_TH \t:" + String(MIN_HUMIDITY_TH), INFO);
  message("CHECK_INTERNET_CONNECT \t:" + String(CHECK_INTERNET_CONNECT), INFO);
  message("DHTesp_MODEL \t:" + String(dhtMain.getModel()), INFO);
  message("HostName: " + WiFi.hostname() + " |Ch: " + String(WiFi.channel()) + " |RSSI: " + WiFi.RSSI() + " |MAC: " + WiFi.macAddress() + " \t Flash Chip Id/Size/Speed/Mode: " + String(ESP.getFlashChipId()) + "/" + String(ESP.getFlashChipSize()) + "/" + String(ESP.getFlashChipSpeed()) + "/" + String(ESP.getFlashChipMode()), INFO);
  message("SdkVersion: " + String(ESP.getSdkVersion()) + "\tCoreVersion: " + ESP.getCoreVersion() + "\tBootVersion: " + ESP.getBootVersion() + "\t CpuFreqMHz: " + String(ESP.getCpuFreqMHz()) + " \tBootMode: " + String(ESP.getBootMode()) + "\tSketchSize: " + String(ESP.getSketchSize()) + "\tFreeSketchSpace: " + String(ESP.getFreeSketchSpace()), INFO);
  //message("getResetReason: " + ESP.getResetReason() + " |getResetInfo: " + ESP.getResetInfo() + " |Address : " + getAddressString(insideThermometer[0]), INFO);
}
