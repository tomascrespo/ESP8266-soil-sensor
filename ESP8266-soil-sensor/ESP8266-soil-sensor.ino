#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
//#include <PubSubClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define DEBUG

#define WIFI_CONNECTION_MAX_TIME 30 // seconds (low precision)
#define WIFI_SSID "VerdePistacho"
#define WIFI_PASSWORD "mysecretWIFIpassword"
#define MY_LAN_IP "192.168.2.175"  // Static IP to reduce WiFi packets. Not used yet

#define MQTT_CONNECTION_TRIES 3
#define MQTT_BROKER "192.168.2.245" // "ingeniolabs.com" // "ingeniolabs.com"  
#define MQTT_PORT 1883
#define MQTT_USER "tomascrespo"
#define MQTT_PASSWORD "mysecretMQTTpassword"

#define SLEEP_SECONDS 600  // 10 minutes

// SoftwareSerial is used to create a second serial port, which will be deidcated to RS485
// The built-in serial port remains available for flashing and debugging
#define RX_PIN D1 // D7  // Serial Receive pin
#define TX_PIN D2 //D6  // Serial Transmit pin
#define SENSOR_READ_MAX_TRIES 12

#define FET_GATE_PIN D7 

SoftwareSerial rs485 = SoftwareSerial(RX_PIN, TX_PIN);

#define SENSOR_FRAME_SIZE 9  // Including moisture and temperature (it will be 11 if it would include conductivity)
#define SENSOR_WAITING_TIME 1000 // Max time to wait for sensor response
#define sensorID 0x01
#define sensorFunction 0x03
#define sensorByteResponse 0x06


// RS485 Byte Address Request to Sensor
unsigned char querySensor1[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B };  // Read 2 points (humidity and temperature) of ID 1
unsigned char querySensor2[8] = { 0x02, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x38 };  // Read 2 points (humidity and temperature) of ID 2
unsigned char byteResponse[9] = {};

float moisture1, moisture2, temperature1, temperature2;
int analogValue = 0;
float batteryVoltage, batteryPercentage;

unsigned long lastMillis = 0;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);
Adafruit_MQTT_Publish feed_sensor1_temp = Adafruit_MQTT_Publish(&mqtt, "soil_sensor/sensor1/temperature");
Adafruit_MQTT_Publish feed_sensor1_moist = Adafruit_MQTT_Publish(&mqtt, "soil_sensor/sensor1/moisture");
Adafruit_MQTT_Publish feed_sensor2_temp = Adafruit_MQTT_Publish(&mqtt, "soil_sensor/sensor2/temperature");
Adafruit_MQTT_Publish feed_sensor2_moist = Adafruit_MQTT_Publish(&mqtt, "soil_sensor/sensor2/moisture");
Adafruit_MQTT_Publish feed_battery_analog = Adafruit_MQTT_Publish(&mqtt, "soil_sensor/battery/analog_read");
Adafruit_MQTT_Publish feed_battery_voltage = Adafruit_MQTT_Publish(&mqtt, "soil_sensor/battery/voltage");
Adafruit_MQTT_Publish feed_battery_capacity = Adafruit_MQTT_Publish(&mqtt, "soil_sensor/battery/capacity");

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();


void setup_WiFi() {
  // We start by connecting to a WiFi network
  #ifdef DEBUG
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);
  #endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  
  int wifi_tries = WIFI_CONNECTION_MAX_TIME;

  while ((WiFi.status() != WL_CONNECTED) && (wifi_tries > 0)) {    
    delay(1000);
    wifi_tries--;
    #ifdef DEBUG
      Serial.print(".");
    #endif
  }

  if (WiFi.status() != WL_CONNECTED) {
    #ifdef DEBUG
      Serial.println("");
      Serial.print("I couldn't connect to ");
      Serial.println(WIFI_SSID);
    #endif
    for (int i =0; i<10;i++) { // Some blinking to warning that there were not WiFi connection
      digitalWrite(LED_BUILTIN, !LOW);  // Led OFF
      delay(300);
      digitalWrite(LED_BUILTIN, !HIGH);  // Led ON
      delay(300);
    }
    
    lets_sleep();
    } else {
      delay(4000); // Wait for DHCP @todo change to static IP @todo adjust, reduce
      #ifdef DEBUG
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
      #endif
    }  
  randomSeed(micros()); // initializes the pseudo-random number generator (for what?)
}


// This function runs once on startup
void setup() {

  // Pin setup    
  pinMode(LED_BUILTIN, OUTPUT);      // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, !HIGH);  // Led ON
  pinMode(FET_GATE_PIN, OUTPUT);
  digitalWrite(FET_GATE_PIN, HIGH);  // 5v step up booster ON (which powers the sensors) and RS485 module ON
  delay(100); // Pin configuration recommended delay

  #ifdef DEBUG
    Serial.begin(115200);  // Initialize USB serial port for debuggin
    while (!Serial) {
      ;  // wait for serial port to connect. Needed for Native USB only
    }
    Serial.println("USB-Serial initialized");
  #endif

  // WiFi
  setup_WiFi();

  rs485.begin(4800);  // Initialize software serial serial port for RS485 module
  #ifdef DEBUG
    Serial.println("RS485 Initialized");
  #endif
}

//uint32_t x = 0;

// This function runs over and over again in a continuous loop
void loop() {

  // Send query request to sensor
  //rs485.write(byteRequest, 8);
  //delay(1000);
  //int bytesToRead = rs485.available();
  //Serial.println("Bytes to read: " + (String)bytesToRead); // How many bytes are pending of been read

  // Query soil sensors
  read_sensor(1, SENSOR_READ_MAX_TRIES); 
  read_sensor(2, SENSOR_READ_MAX_TRIES); 
  //rs485.write(querySensor1, 8);
  //readSensorResponse(1);
  //delay(2000);
  //rs485.write(querySensor2, 8);
  //readSensorResponse(2);

  // Query battery voltage six times and get average
  analogValue = 0;
  analogValue = analogRead(A0);
  delay(200);
  analogValue += analogRead(A0);
  delay(200);
  analogValue += analogRead(A0);
  delay(200);
  analogValue += analogRead(A0);
  delay(200);
  analogValue += analogRead(A0);
  delay(200);
  analogValue += analogRead(A0);
  analogValue = analogValue / 6; // Average of six reads

  Serial.println("Battery voltage: " + (String)analogValue);
  Serial.println();

  MQTT_connect();
  // Now we can publish values
  Serial.print("\nSending sensor1 temperature " + (String)temperature1 + "...");
  if (!feed_sensor1_temp.publish(temperature1)) Serial.println("Failed");
  else Serial.print("OK!");

  Serial.print("\nSending sensor1 moisture " + (String)moisture1 + "...");
  if (!feed_sensor1_moist.publish(moisture1)) Serial.println("Failed");
  else Serial.print("OK!");

  Serial.print("\nSending sensor2 temperature " + (String)temperature2 + "...");
  if (!feed_sensor2_temp.publish(temperature2)) Serial.println("Failed");
  else Serial.print("OK!");

  Serial.print("\nSending sensor2 moisture " + (String)moisture2 + "...");
  if (!feed_sensor2_moist.publish(moisture2)) Serial.println("Failed");
  else Serial.print("OK!");

  Serial.print("\nSending sensor battery analog read " + (String)analogValue + "...");
  if (!feed_battery_analog.publish(analogValue)) Serial.println("Failed");
  else Serial.print("OK!");

  // If analog read is 1024 (maximum) voltage is supposed to be 4.2v (maximum)
  batteryVoltage = (analogValue / 1000.0) * 4.2;  // Calibration: When my battery is fully charged (4.2v) analog input (A0) is 1000
  Serial.print("\nSending sensor battery voltage " + (String)batteryVoltage + "...");
  if (!feed_battery_voltage.publish(batteryVoltage)) Serial.println("Failed");
  else Serial.print("OK!");

  batteryPercentage = getBatteryPercentage(analogValue);
  Serial.print("\nSending sensor battery percentage " + (String)batteryPercentage + "...");
  if (!feed_battery_capacity.publish(batteryPercentage)) Serial.println("Failed");
  else Serial.print("OK!");

  delay(3000);  // To allow finish publish
  lets_sleep();
}

void lets_sleep(){
  #ifdef DEBUG
    Serial.println("Let's sleep");
  #endif
  digitalWrite(LED_BUILTIN, !LOW); // Led OFF
  digitalWrite(FET_GATE_PIN, LOW); // 5v step up booster OFF (so, sensors off) and RS485 module OFF
  ESP.deepSleep(SLEEP_SECONDS * 1000000);
}

// Query the sensor
// @todo You should calculate the CRC to automatize the generation of the query based on sensor id
void read_sensor(int sensor_id, int max_retries) {
  bool success = false;

  while ((!success) && (max_retries > 0)) {
    max_retries--;

    #ifdef DEBUG
      Serial.print("Asking to sensor ");
      Serial.print(sensor_id);
    #endif

    while (Serial.read() >= 0) {} // Discard any input chars before asking

    if (sensor_id == 1) rs485.write(querySensor1, 8);
    else rs485.write(querySensor2, 8);

    // Wait for sensor to response
    unsigned long resptime = millis(); // Returns the number of milliseconds since the Arduino board began running the current program
    while ((rs485.available() < SENSOR_FRAME_SIZE) && ((millis() - resptime) < SENSOR_WAITING_TIME)) { // Repeat until we have full response or waiting time is off
      delay(100);
      #ifdef DEBUG
        Serial.print(".");
      #endif
    }

    // Read response
    for (int n = 0; n < SENSOR_FRAME_SIZE; n++) {
      byteResponse[n] = rs485.read();
    }

    // Check response
    //if (byteResponse[0] != sensor_id && byteResponse[1] != sensorFunction && byteResponse[2] != sensorByteResponse) {
    if (byteResponse[0] != sensor_id) {      
      #ifdef DEBUG
        Serial.println("SENSOR FAILED!");
      #endif      
      delay(200);
    } else {
      success = true;
      #ifdef DEBUG
        String responseString;
        for (int j = 0; j < SENSOR_FRAME_SIZE; j++) {
          responseString += byteResponse[j] < 0x10 ? " 0" : " ";
          responseString += String(byteResponse[j], HEX);
          responseString.toUpperCase();
        }  
        Serial.println("");
        Serial.println("Sensor " + (String)sensor_id + " response: " + responseString);
      #endif      
    }
  }

  // Conversion of byte response to data
  if (byteResponse[0] == 1) {
    moisture1 = sensorValue((int)byteResponse[3], (int)byteResponse[4]) * 0.1;
    temperature1 = sensorValue((int)byteResponse[5], (int)byteResponse[6]) * 0.1;
    #ifdef DEBUG
      // Print the data on Serial Monitor
      Serial.println("Sensor " + (String)byteResponse[0] + " Moisture: " + (String)moisture1 + " %  Temperature: " + (String)temperature1 + " °C");
    #endif
  } else if (byteResponse[0] == 2)  {
    moisture2 = sensorValue((int)byteResponse[3], (int)byteResponse[4]) * 0.1;
    temperature2 = sensorValue((int)byteResponse[5], (int)byteResponse[6]) * 0.1;
    #ifdef DEBUG
      // Print the data on Serial Monitor
      Serial.println("Sensor " + (String)byteResponse[0] + " Moisture: " + (String)moisture2 + " %  Temperature: " + (String)temperature2 + " °C");
    #endif
  } 
    else if (sensor_id == 1) moisture1 = temperature1 = NAN;
    else if (sensor_id == 2) moisture2 = temperature2 = NAN; // I did not get to read any value  

}

// Previous function, NOT USED NOW
void readSensorResponse(int id) {
  // Wait for sensor to response
  unsigned long resptime = millis(); // Returns the number of milliseconds since the Arduino board began running the current program
  while ((rs485.available() < SENSOR_FRAME_SIZE) && ((millis() - resptime) < SENSOR_WAITING_TIME)) {
    delay(1);
  }

  while (rs485.available()) {
    for (int n = 0; n < SENSOR_FRAME_SIZE; n++) {
      byteResponse[n] = rs485.read();
    }

    if (byteResponse[0] != sensorID && byteResponse[1] != sensorFunction && byteResponse[2] != sensorByteResponse) {
      #ifdef DEBUG
        Serial.println("SENSOR FAILED!");
      #endif
      return;
    }
  }

  String responseString;
  for (int j = 0; j < SENSOR_FRAME_SIZE; j++) {
    responseString += byteResponse[j] < 0x10 ? " 0" : " ";
    responseString += String(byteResponse[j], HEX);
    responseString.toUpperCase();
  }
  #ifdef DEBUG
    Serial.println(responseString);
  #endif

  // Conversion of byte response to data
  if (byteResponse[0] == 1) {
    moisture1 = sensorValue((int)byteResponse[3], (int)byteResponse[4]) * 0.1;
    temperature1 = sensorValue((int)byteResponse[5], (int)byteResponse[6]) * 0.1;
    #ifdef DEBUG
      // Print the data on Serial Monitor
      Serial.println("Sensor " + (String)byteResponse[0] + " Moisture: " + (String)moisture1 + " %  Temperature: " + (String)temperature1 + " °C");
    #endif
  } else if (byteResponse[0] == 2)  {
    moisture2 = sensorValue((int)byteResponse[3], (int)byteResponse[4]) * 0.1;
    temperature2 = sensorValue((int)byteResponse[5], (int)byteResponse[6]) * 0.1;
    #ifdef DEBUG
      // Print the data on Serial Monitor
      Serial.println("Sensor " + (String)byteResponse[0] + " Moisture: " + (String)moisture2 + " %  Temperature: " + (String)temperature2 + " °C");
    #endif
  } 
    else if (id == 1) moisture1 = temperature1 = NAN;
    else if (id == 2) moisture2 = temperature2 = NAN; // I did not get to read any value  
}

int sensorValue(int x, int y) {
  int t = 0;
  t = x * 256;
  t = t + y;

  return t;
}


// Connect to MQTT broker using Adafruit library
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  #ifdef DEBUG
    Serial.print("Connecting to MQTT... ");
  #endif

  uint8_t retries = MQTT_CONNECTION_TRIES;
  while ((ret = mqtt.connect()) != 0) {  // connect will return 0 for connected
    #ifdef DEBUG
      Serial.println(mqtt.connectErrorString(ret));
      Serial.println("Retrying MQTT connection in 5 seconds...");
    #endif
    mqtt.disconnect();
    delay(5000);  // Wait 5 seconds between reconnections
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      return;
      //while (1)
      //  ;
    }
  }
  #ifdef DEBUG
    Serial.println("MQTT Connected!");
  #endif
}

float getBatteryPercentage(float analogRead){
  float bat_percent;
  if (analogRead >= 1000) 
    bat_percent = 100;
  else if (analogRead >= 900)    // 90-100% range
    bat_percent = map(analogRead, 900, 1000, 90, 100);
  else if (analogRead >= 800)    // 10-90% range
    bat_percent = map(analogRead, 800, 900, 10, 90);
  else if (analogRead >= 580)    // 0-10% range
    bat_percent = map(analogRead, 580, 800, 0, 10);
  else bat_percent = 0;  
  return bat_percent;
}
