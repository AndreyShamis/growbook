
String getValue(const String &data, const char &separator, const int &index)
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


/**
  Keep type of mesages
*/
//static inline char *stringFromLogType(enum LogType lt)
static const char *stringFromLogType(const enum LogType &lt) {
  static const char *strings[] = {"INFO", "WARN", "ERROR", "PASS", "FAIL", "CRITICAL", "DEBUG"};
  return strings[lt];
}


/**
   Get Temperature
*/
const float getTemperature(DallasTemperature &s, const int &dev/*=0*/) {
  //message("Requesting device " + String(dev), DEBUG);
  s.setWaitForConversion(false);   // makes it async
  s.requestTemperatures();
  s.setWaitForConversion(true);    // makes it async
  return s.getTempCByIndex(dev);
  //return sensor.getTempC(insideThermometer[dev]);
}



/**
   print temperature to serial
*/
void printTemperatureToSerial(DallasTemperature &s) {
  short int dc = s.getDeviceCount();
  short int i = 0;
  for (i = 0 ; i < dc; i++) {
    //message("Temperature[" + String(i) + "] C: " + String(getTemperature(s, i)), INFO);
    Serial.println("INFO: Temperature[" + String(i) + "] C: " + String(getTemperature(s, i)));
  }
}

/**
 *  Convert Dallas Address to String
*/
const String getAddressString(const DeviceAddress &deviceAddress) {
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
const String get_thermometers_addr(DallasTemperature &s) {
  String ret_data = "[";
  short int i = 0;
  short int dev_counter = s.getDeviceCount();
  for (i = 0; i < dev_counter; i++) {
    ret_data = ret_data + String("\"") + String(getAddressString(insideThermometer[i])) + String("\" , ");
  }
  ret_data = ret_data + "]";
  return ret_data;
}

/**
 * 
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
void save_setting(const char* fname, const String &value) {
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

///**
//*/
//String build_index() {
//
//  String ret_js = String("") + "load = \n{" +
//                  "'internet_access': '" + String(internet_access) + "'," +
//                  "'current_temperature_0': '" + String(current_temp[0]) + "'," +
//                  "'current_temperature_1': '" + String(current_temp[1]) + "'," +
//                  "'current_temperature_2': '" + String(current_temp[2]) + "'," +
//                  "'current_temperature_3': '" + String(current_temp[3]) + "'," +
//                  "'flash_chip_id': '" + String(ESP.getFlashChipId()) + "'," +
//                  "'flash_chip_size': '" + String(ESP.getFlashChipSize()) + "'," +
//                  "'flash_chip_speed': '" + String(ESP.getFlashChipSpeed()) + "'," +
//                  "'flash_chip_mode': '" + String(ESP.getFlashChipMode()) + "'," +
//                  "'core_version': '" + ESP.getCoreVersion() + "'," +
//                  "'sdk_version': '" + String(ESP.getSdkVersion()) + "'," +
//                  "'boot_version': '" + ESP.getBootVersion() + "'," +
//                  "'boot_mode': '" + String(ESP.getBootMode()) + "'," +
//                  "'cpu_freq': '" + String(ESP.getCpuFreqMHz()) + "'," +
//                  "'mac_addr': '" + WiFi.macAddress() + "'," +
//                  "'wifi_channel': '" + String(WiFi.channel()) + "'," +
//                  "'rssi': '" + WiFi.RSSI() + "'," +
//                  "'sketch_size': '" + String(ESP.getSketchSize()) + "'," +
//                  "'free_sketch_size': '" + String(ESP.getFreeSketchSpace()) + "'," +
//                  "'temperature_precision': '" + String(TEMPERATURE_PRECISION) + "'," +
//                  "'time_str': '" + timeClient.getFormattedTime() + "'," +
//                  "'time_epoch': '" + timeClient.getEpochTime() + "'," +
//                  "'hostname': '" + WiFi.hostname() + "'" +
//                  "};\n";
//  String ret = String("") + "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><title>Load Info</title></head>" +
//               " <script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.0/jquery.min.js'></script>\n" +
//               " <script src='http://tm.anshamis.com/js/heater.js'></script>\n" +
//               " <link rel='stylesheet' type='text/css' href='http://tm.anshamis.com/css/heater.css'>\n" +
//               "<body><script>" + ret_js + "</script>\n" +
//               "<div id='content'></div>" +
//               "<script>\n " +               "$(document).ready(function(){ onLoadPageLoad(); });</script>\n" +
//               "</body></html>";
//  return ret;
//}


///**
//   Start WEB server
//*/
//void server_start() {
//  server.on("/", handleRoot);
//  server.on("/inline", []() {
//    server.send(200, "text/plain", "this works as well");
//  });
//  //  server.on("/el", []() {
//  //    enableLoad();
//  //    loadMode = MANUAL;
//  //    handleRoot();
//  //  });
//  server.onNotFound(handleNotFound);
//  message("Staring HTTP server...", INFO);
//  server.begin();
//  message("HTTP server started", PASS);
//}



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

///**
//  WEB Server function
//*/
//void handleNotFound() {
//  String message = "File Not Found\n\n";
//  message += "URI: " + server.uri() + "\nMethod: ";
//  message += (server.method() == HTTP_GET) ? "GET" : "POST";
//  message += "\nArguments: ";
//  message += server.args() + "\n";
//  for (uint8_t i = 0; i < server.args(); i++) {
//    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
//  }
//  server.send(404, "text/plain", message);
//}
//
///**
//  WEB Server function
//*/
//void handleRoot() {
//
//  //  for (uint8_t i=0; i<server.args(); i++){
//  //    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
//  //  }
//  //  message += server.client();
//  String message = build_index();
//  server.send(200, "text/html", message);
//}
