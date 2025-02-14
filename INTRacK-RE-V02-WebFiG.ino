/*
 ****************************************
 * INTrackG v0.2 RE tester v0.3
 * program by WATT
 * 2022-10-29
 * add datalog
 **************************************** 
 * Arduino 1.8.13
 * library:
 * - I/O expander I2C -
 * PCF8574 library by Reef version 2.2.2
 * https://github.com/xreef/PCF8574_library
 * - Acc Gyro -
 * SparkFun_LSM6DS3_Arduino_Library version 1.0.1
 * https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
 * - I/O expander I2C -
 * I2C MCP23008 by Rob Tillaart version 0.1.0
 * https://github.com/RobTillaart/MCP23008
 * - ADC I2C 4CH 16bit -
 * MCP342x ADC library by Steve Marple
 * https://github.com/stevemarple/MCP342x
 * - UART1 for ESP32 -
 * note: chang uart pin in ESP32 Hardware serial file 
 * C:\Users\XXXX\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\cores\esp32\HardwareSerial.cpp
 **************************************** 
 */
#include "Arduino.h"
#include "PCF8574.h"
//#include <MPU9250_WE.h>
#include <Wire.h>
#include "MCP23008.h"
#include <MCP342x.h>
#include "SparkFunLSM6DS3.h"

#include "INTRacKWebFig.h"

#define DEBUG
#define WIFI_MQTT //<- comment this line to use 4G
#define BUZZER_MOD //<- uncomment this line to enable buzzer mode

#ifdef WIFI_MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#endif

/* define for SD card log */

/* define pin for SD card */
#define SD_CS_PIN 5
bool sd_ok_flag = false; 

/* SD card include */
#include "FS.h"
#include "SD.h"
#include "SPI.h"

/* define pin for I/O Expander */
#define RED_LED_PIN P4      //PCF8574 0x20 pin P4
#define ORANGE_LED_PIN P6   //PCF8574 0x20 pin P4
#define BLUE_LED_PIN P5     //PCF8574 0x20 pin P4

#define EMERGENCY_SW_PIN 2  //MCP23008 0x22 pin 2
#define DOOR_SW_PIN 1       //MCP23008 0x22 pin 1
#define GYRO_INT_PIN 4      //MCP23008 0x22 pin 3
#define ACC_SEN_PIN 3       //MCP23008 0x22 pin 3

/* define pin for SIM7600 */
#define RXD2 16
#define TXD2 17
#define S76_PWRKEY 32
#define S76_SLEEP 15
#define S76_PWR_EN 27
#define SIM7600_SERIAL Serial2

/* define pin for Analog Read */
#define FUEL_SEN channel3
#define VIN_SEN channel2
#define PT100_SEN channel4
#define BATT_SEN channel1

/* define pin for Card reader */
/*First, change UART1 Pins definition in file*/
/*C:\Users\XXXX\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\cores\esp32\HardwareSerial.cpp */
#define RXD1 26   
#define TXD1 25   
/* define pin for another io */
#define BUZZER_PIN 0

/* define user parameter */
#define SIM_MSG_TIMEOUT 1000
#define SIM_INIT_TIMEOUT 60000
#define SIM_MSG_LEN 256
#define PAYLOAD_LEN 512
#define SIM_ACK_LEN 256

//#define SEND_INTERVAL 15000
#define LED_BLINK_INTERVAL 500

#define TIMER_PRESCALER 80
#define TIMER_INTERVAL 500000 //1 second is 1000000us

unsigned long last_time, now_time, last_io_time;
unsigned long last_red_led_time, last_orange_led_time, last_blue_led_time;
uint32_t mqtt_counter = 0;

int SEND_INTERVAL; 

#ifdef WIFI_MQTT
/* WiFi - MQTT setting */
char ssid[] = ".@Vehicle01";                       // CONFIG
char password[] = "intit2021";                     // CONFIG
char mqtt_server[] = "172.22.1.22";                // CONFIG
int mqtt_port = 1883;                             // CONFIG
char mqtt_topic[] = "INTR.IoT";                   // CONFIG
WiFiClient intClient;
PubSubClient client(intClient);
#else
/* 4G - MQTT setting */
char mqtt_broker[] = "tcp://int.in.th:18883"; // <-- use this one
//char mqtt_broker[] = "tcp://165.22.255.3:1883"; // my broker
//char mqtt_broker[] = "tcp://cctv4rent.thddns.net:7794"; //**not work
char mqtt_topic[] = "intrack/vehiclestate";
#endif

/* massage payload */
char payload_event[] = "WillDeleteLater";
char payload_fleet[] = "WillDeleteLater";
char payload_id[] = "Vehicle01";                          // CONFIG
uint16_t payload_seq = 0;
char payload_time[] = "2021-12-31T13:00:00.000Z";
char payload_card_reader[] = "00000000"; //53000331
float payload_vin = 0;
float payload_vbat = 0;
char payload_engine[] = "OFF";
float payload_speed = 0;
float payload_direction = 0;
float payload_fuel = 0;
double payload_latitude = 0;
double payload_longitude = 0;
char payload_door[] = "CLOSE";
float payload_temp = 0;
float payload_acc_x, payload_acc_y, payload_acc_z;
float payload_gyr_x, payload_gyr_y, payload_gyr_z;
char payload_emer[] = "FALSE";

char payload[PAYLOAD_LEN];
char sim_msg[SIM_MSG_LEN];
char sim_ack[SIM_ACK_LEN];

/* even flag */
boolean acc_on_flag = false;
boolean door_opened_flag = false;
boolean emergency_on_flag = false;
boolean driver_license_flag = false;
boolean led_red_on = false;
boolean led_orange_on = false;
boolean led_blue_on = false;
boolean led_red_blink = false;
boolean led_orange_blink = false;
boolean led_blue_blink = false;

boolean initial_state = true;
boolean engine_start_state = false;
boolean validation_state = false;

/* expansion I/O setting */
PCF8574 ex_io(0x20);
MCP23008 MCP1(0x22);

/* ADC setting */
uint8_t adc_address = 0x68;
MCP342x adc = MCP342x(adc_address);

/* acc gyro setting */
LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B

/**************** START USER FUNCTION *****************/

void initSDCard(void)
{
  if(!SD.begin(SD_CS_PIN)){
    Serial.println("Card Mount Failed!!!");
    sd_ok_flag = false;
  } else {
    Serial.println("Card Mount Successful");
    sd_ok_flag = true;  
  }
}

void writeLog(void)
{
  /* if SD card mount successful */
  if(sd_ok_flag){
    char filename[] = "/20yy-mm-dd.txt";
    /* create 1 file per day, 368 byte per log 3 Mbyte per file */
    /* payload_time[] = "2021-12-31T13:00:00.000Z" */
    for(int i=0;i<10;i++){
      filename[i+1] = payload_time[i]; // using date for filename
    }
    Serial.print("file name = ");
    Serial.println(filename);
    File file = SD.open(filename, FILE_APPEND);
    if(file){
      if(file.println(payload)){
        Serial.println("File written");
      } else {
        Serial.println("Write failed");
      }
      file.close();      
    } else {
      Serial.println("Failed to open file for writing");
    }
  } else {
    Serial.println(" NO SD CARD ");
  }
  return;
}

void Gyro_Init(void){
  Wire.begin();
  myIMU.begin();
  delay(200);
}

void Expanded_IO_Init(void){
  Serial.print("Expanded I/O Init..");
  // Set pinMode to INPUT
  MCP1.begin(21, 22); //SDA, SCL
  MCP1.pinMode(DOOR_SW_PIN, INPUT);
  MCP1.pinMode(EMERGENCY_SW_PIN, INPUT);
  MCP1.pinMode(ACC_SEN_PIN, INPUT);
  MCP1.pinMode(GYRO_INT_PIN, INPUT);
  MCP1.pinMode(BUZZER_PIN, OUTPUT);
  // Set pinMode to OUTPUT
  ex_io.pinMode(RED_LED_PIN, OUTPUT); //RED LED
  ex_io.pinMode(ORANGE_LED_PIN, OUTPUT); //ORANGE LED
  ex_io.pinMode(BLUE_LED_PIN, OUTPUT); //BLUE LED
  if(ex_io.begin()){
    Serial.println("[OK]");
  }
  else{
    Serial.println("[FAIL]");
  }
  Serial.print("ACC_SEN_PIN = ");
  Serial.println(MCP1.digitalRead(ACC_SEN_PIN));
  Serial.print("DOOR_SW_PIN = ");
  Serial.println(MCP1.digitalRead(DOOR_SW_PIN));
  if(MCP1.digitalRead(ACC_SEN_PIN) == 0){
    initial_state = false;
  }
  last_io_time = millis();
}

void validate_license(void){
  
}

void read_ex_io(void){
  //to do emergency sw should be in interrupt mode, or provide direct pin to esp32
  //to change status of emergency flag must do manualy
  if(MCP1.digitalRead(EMERGENCY_SW_PIN)==LOW){ 
    emergency_on_flag = true;    
    strcpy(payload_emer, "TRUE");  
  }
  else{
//    emergency_on_flag = false;
//    strcpy(payload_emer, "FALSE");
  }
  if(MCP1.digitalRead(DOOR_SW_PIN) == 1){
    door_opened_flag = true;
    strcpy(payload_door, "OPEN");  
  }
  else{
    door_opened_flag = false;
    strcpy(payload_door, "CLOSE");
  }
  if(MCP1.digitalRead(ACC_SEN_PIN) == 0){
    acc_on_flag = true;
    strcpy(payload_engine,"ON");
  }
  else{
    acc_on_flag = false;
    strcpy(payload_engine,"OFF");;
  }  
}

void led_loop(void){
  /* red led */
  if(led_red_on){
    if(led_red_blink){
      if((millis() - last_red_led_time) > LED_BLINK_INTERVAL){
        last_red_led_time = millis();
        ex_io.digitalWrite(RED_LED_PIN, !ex_io.digitalRead(RED_LED_PIN));  
      }
    }
    else{
      ex_io.digitalWrite(RED_LED_PIN, LOW);
    }
  }
  else{
    ex_io.digitalWrite(RED_LED_PIN, HIGH);
  }
  /* orange led */
  if(led_orange_on){
    if(led_orange_blink){
      if((millis() - last_orange_led_time) > LED_BLINK_INTERVAL){
        last_orange_led_time = millis();
        ex_io.digitalWrite(ORANGE_LED_PIN, !ex_io.digitalRead(ORANGE_LED_PIN));  
      }
    }
    else{
      ex_io.digitalWrite(ORANGE_LED_PIN, LOW);
    }
  }
  else{
    ex_io.digitalWrite(ORANGE_LED_PIN, HIGH);
  }
  /* blue led */
  if(led_blue_on){
    if(led_blue_blink){
      if((millis() - last_blue_led_time) > LED_BLINK_INTERVAL){
        last_blue_led_time = millis();
        ex_io.digitalWrite(BLUE_LED_PIN, !ex_io.digitalRead(BLUE_LED_PIN));  
      }
    }
    else{
      ex_io.digitalWrite(BLUE_LED_PIN, LOW);
    }
  }
  else{
    ex_io.digitalWrite(BLUE_LED_PIN, HIGH);
  }
}

void Card_Init(void){
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
}

void ADC_Init(void){
  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms
  
  // Check device present
  Wire.requestFrom(adc_address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(adc_address, HEX);
    while(1);
  }  
}

void parseLicenseCard(void){
  if(Serial1.available()){
    Serial1.readBytes(sim_ack, SIM_ACK_LEN);
    Serial.println(sim_ack);
    char* index = strchr(sim_ack,'+');
    if(index != NULL){
      #ifdef BUZZER_MOD
        buzzer_off();
      #endif
      led_blue_blink = false; 
      for(int i=0;i<8;i++){
        if(*(index + 43 + i) != ' '){
          *(payload_card_reader + i) = *(index + 43 + i);
        }
        else{
          break;
        }
      }
    }
  }
}

/* get analog input */
/*
#define FUEL_SEN 36
#define VIN_SEN 39
#define PT100_SEN 34
#define BATT_SEN 35
*/
void getAnalogSensor(void){
  long adc_value = 0;
  float vref = 3.3;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::VIN_SEN, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, adc_value, status);
//  if (err) {
//    Serial.print("Convert error: ");
//    Serial.println(err);
//  }
  payload_vin = (adc_value/32767.0)*vref*7.14; //13.33
  adc_value = 0;
  err = adc.convertAndRead(MCP342x::BATT_SEN, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, adc_value, status);
  payload_vbat = (adc_value/32767.0)*vref*(111.0/75.0);
  adc_value = 0;
  err = adc.convertAndRead(MCP342x::FUEL_SEN, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, adc_value, status);
  payload_fuel = (adc_value/32767.0)*100.0;
  adc_value = 0;
  //MCP342x::Config status;
  err = adc.convertAndRead(MCP342x::PT100_SEN, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, adc_value, status);
  payload_temp = (adc_value/32767.0)*vref;
  adc_value = 0;
}

// void gpio_init(void){
//   MCP1.pinMode(BUZZER_PIN, OUTPUT);
// }

void buzzer_on(void){
  MCP1.digitalWrite(BUZZER_PIN, HIGH);
}

void buzzer_off(void){
  MCP1.digitalWrite(BUZZER_PIN, LOW);
}



void getAccGyro(void){
  payload_acc_x = myIMU.readFloatAccelX();
  payload_acc_y = myIMU.readFloatAccelY();
  payload_acc_z = myIMU.readFloatAccelZ();
  payload_gyr_x = myIMU.readFloatGyroX();
  payload_gyr_y = myIMU.readFloatGyroY();
  payload_gyr_z = myIMU.readFloatGyroZ();
    
}

void setPayload(void){
  memset(payload, 0, sizeof(payload));
  sprintf(payload, "{\"fleet\":\"%s\",\"id\":\"%s\",\"seq\":%d,\"time\":\"%s\",\"event\":\"%s\",\"card-reader\":\"%s\",\"vin\":%.1f,\"vbat\":%.1f,\"engine\":\"%s\",\"speed\":%.1f,\"direction\":%.1f,\"fuel\":%.1f,\"latitude\":%.7f,\"longitude\":%.7f,\"door\":\"%s\",\"temp\":%.1f,\"acc-x\":%.7f,\"acc-y\":%.7f,\"acc-z\":%.7f,\"gyr-x\":%.7f,\"gyr-y\":%.7f,\"gyr-z\":%.7f,\"emergency\":\"%s\"}", payload_fleet, objConfig.hardwareID, payload_seq, payload_time, payload_event, payload_card_reader, payload_vin, payload_vbat, payload_engine, payload_speed, payload_direction, payload_fuel, payload_latitude, payload_longitude, payload_door, payload_temp, payload_acc_x, payload_acc_y, payload_acc_z, payload_gyr_x, payload_gyr_y, payload_gyr_z, payload_emer);
}

void parseACK(void){
  while(SIM7600_SERIAL.available()){
     Serial.write(SIM7600_SERIAL.read()); 
  }
}

/* check for OK from Acknowlage Massage */
boolean parseOK(void){
  memset(sim_ack, 0, sizeof(sim_ack));
  unsigned long parse_time = millis();
  /* waiting for incomming masage */
  while(SIM7600_SERIAL.available()){
    SIM7600_SERIAL.readBytes(sim_ack, SIM_ACK_LEN);
    if(strstr(sim_ack, "OK") != NULL){
      return true;     
    }
    else{
      return false;     
    }
  }
  return false;
}

void GPS_Init(void){
  led_red_on = true;
  led_red_blink = true;
  SIM7600_SERIAL.println("AT+CGPS=1");
  delay(1000);
  parseACK();
}
/* get GPS information */
boolean getGPS(void){
  SIM7600_SERIAL.println("AT+CGPSINFO");
  delay(1000);
  if(parseGPS()){
    led_red_blink = false;
    led_loop();
    return true;
  }
  else{
    led_red_blink = true;
    return false;
  }
}

/* read and parse GPS value 
 * +CGPSINFO: 1346.539447,N,10028.842226,E,230122,111321.0,37.4,0.0,115.3
 * +CGPSINFO:[<lat>],[<N/S>],[<log>],[<E/W>],[<date>],[<UTCtime>],[<alt>],[<speed>],[<course>]
*/
boolean parseGPS(void){
  memset(sim_ack, 0, sizeof(sim_ack));
  unsigned long parse_time = millis();
  /* waiting for incomming masage */
  while((millis() - parse_time) < SIM_MSG_TIMEOUT){
    if(SIM7600_SERIAL.available()){
      SIM7600_SERIAL.readBytes(sim_ack, SIM_ACK_LEN);
      #ifdef DEBUG
        Serial.println(sim_ack);
      #endif
      /* parse GPS data from massage */
      if(strstr(sim_ack, "INFO:") != NULL){
        char gps_lat[12], gps_log[13], gps_date[7], gps_time[9], gps_speed[6], gps_course[6];
        char* index_left;
        char* index_right;
        uint32_t len;
        
        /* latitude left boundary */
        index_left = strchr(sim_ack,':');
        if(index_left == NULL){
          #ifdef DEBUG
            Serial.println("GPS DATA ERROR");
          #endif
          return false;
        }
        /* latitude right boundary */
        index_right = strchr(index_left,',');
        if(index_right == NULL){
          #ifdef DEBUG
            Serial.println("GPS DATA ERROR");
          #endif
          return false;
        }  
        len = index_right - index_left;
        if(len <= 1){
          #ifdef DEBUG
            Serial.println("LAT NO DATA");
          #endif
          return false;
        }  
        memcpy(gps_lat, index_left+1, len-1);
        double lat_ddmm = atof(gps_lat);
        double lat_dd = (double)(((int)lat_ddmm)/100);
        double lat_mm = (lat_ddmm - (lat_dd * 100))/60;
        payload_latitude = lat_dd + lat_mm;
        
        /* longitude left boundary */ 
        index_left = index_right + 3;
        /* longitude right boundary */
        index_right = strchr(index_left,',');
        len = index_right - index_left;
        if(len < 1){
          #ifdef DEBUG
            Serial.println("LOG NO DATA");
          #endif
          return false;  }
        memcpy(gps_log, index_left, len);
        double log_dddmm = atof(gps_log);
        double log_ddd = (double)(((int)log_dddmm)/100);
        double log_mm = (log_dddmm - (log_ddd * 100))/60;
        payload_longitude = log_ddd + log_mm;
        #ifdef DEBUG
          char latlong[] = "ll.lllllll,lll.lllllll";
          sprintf(latlong,"%.7f,%.7f", payload_latitude, payload_longitude);
          Serial.println(latlong);
        #endif
        
        /* date left boundary */ 
        index_left = index_right + 3;
        /* date right boundary */
        index_right = strchr(index_left,',');
        len = index_right - index_left;
        if(len < 1){
          #ifdef DEBUG
            Serial.println("DATE NO DATA");
          #endif
          return false;  }
        memcpy(gps_date, index_left, len);
        uint32_t date_ddmmyy = atoi(gps_date);
        uint32_t date_dd = date_ddmmyy/10000;
        uint32_t date_mm = (date_ddmmyy - (date_dd * 10000))/100;
        uint32_t date_yy = date_ddmmyy - ((date_ddmmyy/100)*100); 
      
        /* time left boundary */ 
        index_left = index_right + 1;
        /* time right boundary */
        index_right = strchr(index_left,',');
        len = index_right - index_left;
        if(len < 1){
          #ifdef DEBUG
            Serial.println("NO DATA");
          #endif
          return false;  }
        memcpy(gps_time, index_left, len);
        float time_hhmmss = atof(gps_time);
        uint32_t time_hh = ((uint32_t)time_hhmmss)/10000;
        uint32_t time_mm = (((uint32_t)time_hhmmss) - (time_hh * 10000))/100;
        float time_ss = time_hhmmss - (time_hh * 10000.0) - (time_mm * 100.0);

        /* set time payload */
        sprintf(payload_time,"20%d-%d-%dT%d:%d:%.3fZ", date_yy, date_mm, date_dd, time_hh, time_mm, time_ss);  
      
        /* alt left boundary (skip this value) */ 
        index_left = index_right + 1;
        /* alt right boundary (skip this value)*/
        index_right = strchr(index_left,',');
        
        /* speed left boundary */ 
        index_left = index_right + 1;
        /* speed right boundary */
        index_right = strchr(index_left,',');
        len = index_right - index_left;
        if(len < 1){
          #ifdef DEBUG
            Serial.println("NO DATA");
          #endif
          return false;  }
        memcpy(gps_speed, index_left, len);
        //convert from knot to km/h
        payload_speed = 1.852 * atof(gps_speed); // 1 knot = 1.852 km/h 
      
        /* course left boundary */ 
        index_left = index_right + 1;
        /* course right boundary */
        index_right = strchr(index_left,'\0');
        len = index_right - index_left;
        if(len < 1){
          #ifdef DEBUG
            Serial.println("COURSE NO DATA");
          #endif
          return false;  }
        memcpy(gps_course, index_left, len);
        payload_direction = atof(gps_course);
      }
      else{
        #ifdef DEBUG
          Serial.println("NO ACK");
        #endif 
        return false;     
      }
      return true;
    }
  }
  return false;
}
/***** WiFi-MQTT or 4G-MQTT *****/
#ifdef WIFI_MQTT
/*** WiFi-MQTT ***/
void WiFi_init(){
  delay(10);
  // We start by connecting to a WiFi network

  loadConfiguration(filename, objConfig);
 
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(objConfig.wifiSSID);
  Serial.println(objConfig.wifiPassword); 

  if (objConfig.wifiSSID == "") {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("Connect via default");
  }
  else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(objConfig.wifiSSID, objConfig.wifiPassword);
  }
  
  
//  Serial.println(ssid);
//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    // buzzer_on();
    // delay(1000);
    // buzzer_off();
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void MQTT_init(){
//  client.setServer(mqtt_server, mqtt_port);
  client.setServer(objConfig.mqttServer, objConfig.mqttPort);
  client.setBufferSize(1024);
}

void MQTT_reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    if(WiFi.status() != WL_CONNECTED){
    WiFi_reconnect();
  }
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
//    if (client.connect("intClient")) {
    if (client.connect(objConfig.hardwareID)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
/* Send WiFi-MQTT */
void sendMQTT(void){
  #ifdef DEBUG
    Serial.println(payload);
    Serial.println("data sending");
  #endif
//  client.publish(mqtt_topic, payload);

    client.publish(objConfig.mqttTopic, payload);

  /* reset flag after send data */  
  emergency_on_flag = false;
  strcpy(payload_emer, "FALSE");
}

void WiFi_reconnect(){
  Serial.println("Reconnecting to WIFI network");
  WiFi.disconnect();
  WiFi.reconnect();  
}
#else
/*** 4G-MQTT ***/
/* Initial MQTT */
boolean MQTT_init(void){
  SIM7600_SERIAL.println("AT+CMQTTSTART");
  delay(1000);
  parseACK();
  sprintf(sim_msg,"AT+CMQTTACCQ=0,\"%s\",0,4", payload_id);  
  SIM7600_SERIAL.println(sim_msg);
  delay(1000);
  parseACK();
  SIM7600_SERIAL.println("AT+CMQTTDISC=0,120");
  delay(1000);
  parseACK();
  sprintf(sim_msg, "AT+CMQTTCONNECT=0,\"%s\",60,1", mqtt_broker); 
//  SIM7600_SERIAL.println("AT+CMQTTCONNECT=0,\"tcp://cctv4rent.thddns.net:7794\",60,1");
  SIM7600_SERIAL.println(sim_msg);
  delay(1000);
  parseACK();
  return true;
}
/* Send MQTT */
void sendMQTT(void){
    sprintf(sim_msg, "AT+CMQTTTOPIC=0,%d", strlen(mqtt_topic)); 
    SIM7600_SERIAL.println(sim_msg);
    delay(1000);
    parseACK();
    SIM7600_SERIAL.println(mqtt_topic);
    delay(1000);
    parseACK();
    sprintf(sim_msg,"AT+CMQTTPAYLOAD=0,%d", strlen(payload));  
    SIM7600_SERIAL.println(sim_msg);
    delay(1000);
    SIM7600_SERIAL.println(payload);
    delay(1000);
    parseACK();
    SIM7600_SERIAL.println("AT+CMQTTPUB=0,0,120");
    delay(1000);
    parseACK();  
}
#endif

/* Initial SIM7600 */
boolean SIM7600_init(void){
  SIM7600_SERIAL.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(100);
  pinMode(S76_PWRKEY, OUTPUT);
  digitalWrite(S76_PWRKEY, HIGH);
  delay(1000);
  digitalWrite(S76_PWRKEY, LOW);
  Serial.print("SIM7600 INITING");
  /* waiting for SIM7600 to init */
  led_orange_on = true;
  led_orange_blink = true;
  unsigned long init_start_time = millis();
  while((millis() - init_start_time) < SIM_INIT_TIMEOUT){
    for(int i=0;i<10;i++){
      led_loop();
      Serial.print(".");
      delay(500);      
    }
    SIM7600_SERIAL.println("AT");
    for(int i=0;i<5;i++){
      led_loop();
      Serial.print(".");
      delay(500);      
    }
    if(parseOK()){
      Serial.println("[DONE]");
      led_orange_blink = false;
      led_loop();
      return true;
    }
  }
  Serial.println("[FAIL]");
  return false;    
}
/**************** END USER FUNCTION *****************/

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Expanded_IO_Init();
  Gyro_Init();
  // gpio_init();
  ADC_Init();

//  initWebFig();

//    strlcpy(objConfig.wifiSSID,                  // <- destination
//          doc["wifiSSID"],  // <- source
//          sizeof(objConfig.wifiSSID));         // <- destination's capacity
//
//  char* ssid = objConfig.wifiSSID                       // CONFIG
//  char* password = objConfig.wifiPassword
//  char* mqtt_server = objConfig.mqttServer                // CONFIG
//  int mqtt_port = objConfig.mqttPort                             // CONFIG
//  char mqtt_topic[] = objConfig.mqttTopic
//  char payload_id[] = objConfig.hardwareID
//  SEND_INTERVAL = objConfig.mqttInterval;
//
//  objConfig.mqttUser          // Not Exist
//  objConfig.mqttPassword      // Not Exist

//    strncpy(ssid,objConfig.wifiSSID,sizeof(ssid)); 
//    strncpy(password,objConfig.wifiPassword,sizeof(password)); 
//    strncpy(mqtt_server,objConfig.mqttServer,sizeof(mqtt_server)); 
//    mqtt_port = objConfig.mqttPort;
//    strncpy(mqtt_topic,objConfig.mqttTopic,sizeof(mqtt_topic)); 
//    strncpy(payload_id,objConfig.hardwareID,sizeof(payload_id)); 
//    SEND_INTERVAL = objConfig.mqttInterval;

  initSDCard();
  
  #ifdef BUZZER_MOD
    buzzer_on();
    delay(1000);
    buzzer_off();
    delay(1000);
    buzzer_on();
    delay(1000);
    buzzer_off();
    delay(1000);
  #endif
  while(!SIM7600_init()); //wait for SIM7600 init succesfully
  Card_Init();
  GPS_Init();
  #ifdef WIFI_MQTT
    WiFi_init();
  #endif 

  initWebFig();

  MQTT_init();
  last_time = millis();
}

void loop() {
  led_loop();
  #ifdef WIFI_MQTT
    if (!client.connected()) {
      MQTT_reconnect();
    }
    client.loop();
  #endif
  parseLicenseCard();
  if(millis() - last_io_time > 1000){
    last_io_time = millis();
    read_ex_io();
  }
  if(initial_state && acc_on_flag){
    initial_state = false;
    led_blue_on = true;
    led_blue_blink = true;
    #ifdef BUZZER_MOD
      buzzer_on();
      delay(1000);
      buzzer_off();
    #endif   
  }
  if(!initial_state && !acc_on_flag){
    initial_state = true;
    led_blue_on = false;
    led_blue_blink = false;
  }
  
  /* send interval */
  now_time = millis();
  if(now_time - last_time > objConfig.mqttInterval){
    last_time = now_time;
    getGPS();
    getAnalogSensor();
    getAccGyro();
    setPayload();
    /* write to Log file */
    writeLog();
    
    #ifdef WIFI_MQTT
      sendMQTT();
    #else
      if(mqtt_counter > 50){
        MQTT_init();
        mqtt_counter = 0;
      }
      sendMQTT();
      mqtt_counter++;
    #endif
    payload_seq++;
  }
}
