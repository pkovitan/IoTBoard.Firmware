#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <SPIFFS.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <Hash.h>
  #include <FS.h>
#endif
#include <ESPAsyncWebServer.h>

#include <ArduinoJson.h>

//#include "INTRacKConfig.h"

struct Config {
    int workingMode;
    char wifiSSID[16];
    char wifiPassword[16];
    char mqttServer[32];
    int mqttPort;
    char mqttUser[16];
    char mqttPassword[16];
    char mqttTopic[64];
    int mqttInterval;
    char hardwareID[16];
};

char *filename = "/inputString.txt";  // <- SD library uses 8.3 filenames
Config objConfig;

AsyncWebServer server(80);

// INTRacK Input Parameter
const char* PARAM_WORKINGMODE = "inputWorkingMode";
const char* PARAM_WIFISSID = "inputWifiSSID";
const char* PARAM_WIFIPASSWORD = "inputWifiPassword";
const char* PARAM_MQTTSERVER = "inputMqttServer";
const char* PARAM_MQTTPORT = "inputMqttPort";
const char* PARAM_MQTTUSER = "inputMqttUser";
const char* PARAM_MQTTPASSWORD = "inputMqttPassword";
const char* PARAM_MQTTTOPIC = "inputMqttTopic";
const char* PARAM_MQTTINTERVAL = "inputMqttInterval";
const char* PARAM_HID = "inputHardwareID";

// HTML web page to handle 3 input fields (inputString, inputInt, inputFloat)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>WebFig for WaTTBoard version 1.0</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head><body>
  <form action="/get" target="hidden-form">
    <h2>WEBFiG for INTRacK WATTBoard</h2>
    <hr>
    <br>
    <label for="cars">Choose a car:</label>
      <select name="workingMode" id="workingMode">
        <optgroup label="Select WATTBoard WorkingMode">
          <option value="wifiMode">Wifi Mode</option>
          <option value="lteMode">LTE Mode</option>
        </optgroup>
    </select>
    <br><br>
    WIFI SSID : <input type="text" name="inputWifiSSID" value=%inputWifiSSID%> <br><br>
    WIFI Password : <input type="text" name="inputWifiPassword" value=%inputWifiPassword%> <br><br>
    MQTT Server : <input type="text" name="inputMqttServer" value=%inputMqttServer%> <br><br>    
    MQTT Port : <input type="number" name="inputMqttPort" value=%inputMqttPort%> <br><br>
    MQTT User : <input type="text" name="inputMqttUser" value=%inputMqttUser%> <br><br>    
    MQTT Password : <input type="text" name="inputMqttPassword" value=%inputMqttPassword%> <br><br>    
    MQTT Topic : <input type="text" name="inputMqttTopic" value=%inputMqttTopic%> <br><br>    
    MQTT Interval : <input type="number" name="inputMqttInterval" value=%inputMqttInterval%> <br><br>    
    Hardware ID : <input type="text" name="inputHardwareID" value=%inputHardwareID%> <br><br>   

    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// Replaces placeholder with stored values
String processor(const String& var){
  if (var == "inputWifiSSID") {
    return objConfig.wifiSSID;
  }
  else if (var == "inputWifiPassword") {
    return objConfig.wifiPassword;
  }
  else if (var == "inputMqttServer") {
    return objConfig.mqttServer;
  }
  else if (var == "inputMqttPort") {
    return String(objConfig.mqttPort);
  }
  else if (var == "inputMqttUser") {
    return objConfig.mqttUser;
  }
  else if (var == "inputMqttPassword") {
    return objConfig.mqttPassword;
  }
  else if (var == "inputMqttTopic") {
    return objConfig.mqttTopic;  
  }
  else if (var == "inputMqttInterval") {
    return String(objConfig.mqttInterval);
  }
  else if (var == "inputHardwareID") {
    return objConfig.hardwareID;
  }

  return String();
}


void loadDefaultConfig(char *filename, Config &objConfig) {
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  File file = SPIFFS.open("/inputString.txt", "w");

  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  doc["wifiSSID"] = ".@TWSS";
  doc["wifiPassword"] = "twss2023";
  doc["mqttServer"] = "xxxx";
  doc["mqttPort"] = 0;
  doc["mqttUser"] = "xxxx";
  doc["mqttPassword"] = "xxxx";
  doc["mqttTopic"] = "xxxx";
  doc["mqttInterval"] = 0;
  doc["hardwareID"] = "xxxx";
  
  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
  
}

// Loads the configuration from a file
void loadConfiguration(char *filename, Config &objConfig) {
  // Open file for reading
    
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  File file = SPIFFS.open("/inputString.txt", "r");

  if (!file) {
    Serial.println(F("Failed to read file"));
    return;
  }

  if (file.size() == 0) { 
    loadDefaultConfig(filename,objConfig);
  }

/*      IF File Not Exist Then
 *          CreateFile
 *          Write Default Config
 *      END IF
 */   
 
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  strlcpy(objConfig.wifiSSID,                  // <- destination
          doc["wifiSSID"],  // <- source
          sizeof(objConfig.wifiSSID));         // <- destination's capacity
  strlcpy(objConfig.wifiPassword,                  // <- destination
          doc["wifiPassword"],  // <- source
          sizeof(objConfig.wifiPassword));         // <- destination's capacity
  strlcpy(objConfig.mqttServer,                  // <- destination
          doc["mqttServer"],  // <- source
          sizeof(objConfig.mqttServer));         // <- destination's capacity
  objConfig.mqttPort = doc["mqttPort"];
  strlcpy(objConfig.mqttUser,                  // <- destination
          doc["mqttUser"],  // <- source
          sizeof(objConfig.mqttUser));         // <- destination's capacity
  strlcpy(objConfig.mqttPassword,                  // <- destination
          doc["mqttPassword"],  // <- source
          sizeof(objConfig.mqttPassword));         // <- destination's capacity
  strlcpy(objConfig.mqttTopic,                  // <- destination
          doc["mqttTopic"],  // <- source
          sizeof(objConfig.mqttTopic));         // <- destination's capacity
  objConfig.mqttInterval = doc["mqttInterval"];
  strlcpy(objConfig.hardwareID,                  // <- destination
          doc["hardwareID"],  // <- source
          sizeof(objConfig.hardwareID));         // <- destination's capacity
  
  file.close();
}

// Saves the configuration to a file
void saveConfiguration(char *filename, Config &objConfig) {
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  File file = SPIFFS.open("/inputString.txt", "w");

  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  doc["wifiSSID"] = objConfig.wifiSSID;
  doc["wifiPassword"] = objConfig.wifiPassword;
  doc["mqttServer"] = objConfig.mqttServer;
  doc["mqttPort"] = objConfig.mqttPort;
  doc["mqttUser"] = objConfig.mqttUser;
  doc["mqttPassword"] = objConfig.mqttPassword;
  doc["mqttTopic"] = objConfig.mqttTopic;
  doc["mqttInterval"] = objConfig.mqttInterval;
  doc["hardwareID"] = objConfig.hardwareID;
  
  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();

  delay(1000);

  ESP.restart();
}


void initWebFig() {
  // Initialize SPIFFS
//  #ifdef ESP32
//    if(!SPIFFS.begin(true)){
//      Serial.println("An Error has occurred while mounting SPIFFS");
//      return;
//    }
//  #else
//    if(!SPIFFS.begin()){
//      Serial.println("An Error has occurred while mounting SPIFFS");
//      return;
//    }
//  #endif

//  loadConfiguration(filename, objConfig);

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam(PARAM_WIFISSID)) {
      inputMessage = request->getParam(PARAM_WIFISSID)->value();
      strlcpy(objConfig.wifiSSID,inputMessage.c_str(),sizeof(objConfig.wifiSSID));         // <- destination's capacity
    }
    else {
//      inputMessage = inputMessage & "::"& "No message sent";
    }

    if (request->hasParam(PARAM_WIFIPASSWORD)) {
      inputMessage = request->getParam(PARAM_WIFIPASSWORD)->value();
      strlcpy(objConfig.wifiPassword,inputMessage.c_str(),sizeof(objConfig.wifiPassword));         // <- destination's capacity
    }
    else {
//      inputMessage = inputMessage & "::"& "No message sent";
    }

    if (request->hasParam(PARAM_MQTTSERVER)) {
      inputMessage = request->getParam(PARAM_MQTTSERVER)->value();
      strlcpy(objConfig.mqttServer,inputMessage.c_str(),sizeof(objConfig.mqttServer));         // <- destination's capacity
    }
    else {
//      inputMessage = inputMessage & "::"& "No message sent";
    }

    if (request->hasParam(PARAM_MQTTPORT)) {
      inputMessage = request->getParam(PARAM_MQTTPORT)->value();
      objConfig.mqttPort = inputMessage.toInt();
    }
    else {
//      inputMessage = inputMessage & "::"& "No message sent";
    }

    if (request->hasParam(PARAM_MQTTUSER)) {
      inputMessage = request->getParam(PARAM_MQTTUSER)->value();
      strlcpy(objConfig.mqttUser,inputMessage.c_str(),sizeof(objConfig.mqttUser));         // <- destination's capacity
    }
    else {
//      inputMessage = inputMessage & "::"& "No message sent";
    }

    if (request->hasParam(PARAM_MQTTPASSWORD)) {
      inputMessage = request->getParam(PARAM_MQTTPASSWORD)->value();
      strlcpy(objConfig.mqttPassword,inputMessage.c_str(),sizeof(objConfig.mqttPassword));         // <- destination's capacity
    }
    else {
//      inputMessage = inputMessage & "::"& "No message sent";
    }

    if (request->hasParam(PARAM_MQTTTOPIC)) {
      inputMessage = request->getParam(PARAM_MQTTTOPIC)->value();
      strlcpy(objConfig.mqttTopic,inputMessage.c_str(),sizeof(objConfig.mqttTopic));         // <- destination's capacity
    }
    else {
//      inputMessage = inputMessage & "::"& "No message sent";
    }

    if (request->hasParam(PARAM_MQTTINTERVAL)) {
      inputMessage = request->getParam(PARAM_MQTTINTERVAL)->value();
      objConfig.mqttInterval = inputMessage.toInt();
    }
    else {
//      inputMessage = inputMessage & "::"& "No message sent";
    }

    if (request->hasParam(PARAM_HID)) {
      inputMessage = request->getParam(PARAM_HID)->value();
      strlcpy(objConfig.hardwareID,inputMessage.c_str(),sizeof(objConfig.hardwareID));         // <- destination's capacity
    }
    else {
//      inputMessage = inputMessage & "::"& "No message sent";
    }

    saveConfiguration(filename, objConfig);

    request->send(200, "text/text", inputMessage);
  });
  server.onNotFound(notFound);
  server.begin();
}
