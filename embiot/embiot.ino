#include <WiFi.h>
#include <DHTesp.h> 
#include <PubSubClient.h>

//#define DEBUG_MODE // comment on nominal operation

const int DHT_PIN  = 23;
DHTesp dht22;

const char* WIFI_SSID           = "Huawei_yXD8xy";
const char* WIFI_PASSWORD       = "XNA6SvmG";

const char* MQTT_USER           = "user10"; 
const char* MQTT_CLIENT         = "user10"; 
const char* MQTT_ADDRESS        = "esp-32.zapto.org";


/** MQTT Topics **/
const char* MQTT_TOPIC_CONTROL  = "control";
const char* MQTT_TOPIC_DATA_CRC = "dataCrc";
const char* MQTT_TOPIC_DATA     = "data";

char topicControl[150];
char topicDataCrc[150];
char topicData[150];
/** *** *** *** **/

const uint8_t GENERATOR   = 10; // CRC generator polynomial (x^3 + x^1)
const int LSQ_WINDOW_SIZE = 5;  // the buffer length for the linear squared approximation model

int sampling_time  = 1000;      // sampling period
int temp_threshold = 0;         // temperature threshold
int hum_threshold  = 0;         // humidity threshold

char crc[10];                             // checksum
float temp_vector[LSQ_WINDOW_SIZE] = {0}; // cyclic buffer
int window_index                   = 0;   // cyclic buffer index

volatile unsigned long measPreviousMillis = 0; // timer

bool initialized = false;         // ctrl flag: sent request to server => configuration values are obtained
bool startMeasurements = false;   // ctrl flag: sent CRC => startMeasurements command is obtained 
bool lsqInit = false;             // ctrl flag: start linear squared approximation

void callback(char*, byte*, unsigned int);

WiFiClient wifiClient;
PubSubClient client(MQTT_ADDRESS, 1883, callback, wifiClient);



/* 
  Function to compute the 8-bit CRC of a byte 

  @byte : the input value.
  @generator : the generator value used for the CRC encoding
  @return : returns the CRC of the inputed value
*/
uint8_t crc8(uint8_t byte, const uint8_t generator) {
  uint8_t crc = byte;     

  for(uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80) { // 0x80 = b10000000
      crc = (crc << 1) ^ generator;
    } else {
      crc <<= 1;
    }
  }
  return crc;
}
/* end of crc8 */


/* 
  Function to compute the checksum of a 6-digit number in pairs using the CRC 8-bit algorithm 

  @input : the 6-digit input string
  @generator : the generator value to be used for the CRC encoding
  @output : the CRC checksum string of 9-digits
*/
void compute_crc8(const char* input, uint8_t generator, char* output) {

    char buffer[3];
    int outputIndex = 0;

    for (int i = 0; i < 3; i++) {
        // get the pair of numbers
        strncpy(buffer, &input[i * 2], 2);
        
        // add terminator
        buffer[2] = '\0';

        // convert string to number (byte)
        uint8_t val = (uint8_t)atoi(buffer);

        // compute CRC
        uint8_t crc = crc8(val, generator);

        // fromat as 3-digit number and append to output string
        sprintf(&output[outputIndex], "%03d", crc);
        outputIndex += 3;
    }

    output[9] = '\0';
}
/* end of compute_crc8 */



/* 
  MQTT Callback function 

  @topic : the subscribed topic
  @payload : the payload data bytes
  @length : the length of the payload in bytes
*/
void callback(char* topic, byte* payload, unsigned int length) {
 
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;

  String message(p);
  String topic_str(topic);

  if ( (topic_str == topicData) && (!initialized)) {
    
    // compute the checksum of the received configuration string with 8-bit CRC
    compute_crc8(message.c_str(), GENERATOR, crc);
    
    // extract the configurations values from the configuration string received
    String DDstr = message.substring(0, 2);
    String TTstr = message.substring(2, 4);
    String HHstr = message.substring(4, 6);

    // Convert to numers
    sampling_time  = DDstr.toInt() * 1000;
    temp_threshold = TTstr.toInt();
    hum_threshold  = HHstr.toInt();

    #ifdef DEBUG_MODE 
      Serial.println("[INFO] callback : Configuration parameters ");
      Serial.print("\tSampling Period (s) : ");
      Serial.println(sampling_time);

      Serial.print("\tTemperature Threshold (C) : ");
      Serial.println(temp_threshold);

      Serial.print("\tHumidity Threshold (%) : ");
      Serial.println(hum_threshold);

      Serial.print("[INFO] callback : CRC Checksum : ");
      Serial.println(crc);
    #endif
    
    // update flag into initialized
    initialized = true;
  }
  
  
  if (topic_str == topicDataCrc) {
    if (message == "crcError") {
      
      #ifdef DEBUG_MODE
        Serial.println("[ERROR] callback: crcError");
      #endif
      
      // In case of error in CRC the system cannot initialize
      initialized = false;
    }
  }
  

  if (topic_str == topicControl) {
    if (message == "updateConfig") {
      
      #ifdef DEBUG_MODE
        Serial.println("[STATUS] callback: updateConfig");
      #endif
      
      // when updateConfig is received reset the system 
      initialized = false;
      startMeasurements = false;

    } else if ( (message == "startMeasurements") && (!startMeasurements)) {

      #ifdef DEBUG_MODE
        Serial.println("[STATUS] callback: startMeasurements");
      #endif
      
      // when start measurements is received the system can start sampling the sensor
      startMeasurements = true;
    }
  } 

}/* end of callback */



/* 
  Funtion to estabilish a connection with the MQTT broker 
*/
void mqttConnect() {
  
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("[STATUS] mqttConnect : Attempting MQTT connection");
    
    // Attempt to connect
    if (client.connect(MQTT_CLIENT, "samos", "karlovasi33#")) {
      Serial.println("[INFO] mqttConnect : MQTT connected");
      topicSubscribe();

    } else {
      Serial.print("[ERROR] mqttConnect : rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

}/* end of mqttConnect */



/* 
  Function to subscribe to the MQTT topics 
*/
void topicSubscribe() {
  if(client.connected()) {

    Serial.println("[INFO] topicSubscribe : Subscribe to MQTT topics: ");

    Serial.print("\t");
    Serial.println(topicControl);

    Serial.print("\t");
    Serial.println(topicDataCrc);

    Serial.print("\t");
    Serial.println(topicData);

    // Subscribe to topics
    client.subscribe(topicControl);
    client.subscribe(topicDataCrc);
    client.subscribe(topicData);

    client.loop();
  }  
}/* end of topicSubscribe */



/* 
  Function to estabilish a WiFi connection 

  @ssid : the netwokrs SSID
  @pswd : the networks password  
*/
void wifiConnect(const char* ssid, const char* pswd) {
  Serial.print("[STATUS] wifiConnect : Connecting to SSID : ");
  Serial.println(ssid);

  WiFi.begin(ssid, pswd);
  
  // loop until connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("[STATUS] wifiConnect :  WiFi Connected");

  Serial.print("[INFO] wifiConnect : IP address: ");
  Serial.println(WiFi.localIP());
}/* end of wifiConnect */




/*
  Function to threshold a given value with a given threshold value

  @val : the input value
  @thresh : the threshold value
  @message : the message to display when the value passes the threshold
*/
void threshold(float val, float thresh, const char* message) {
  if (val > thresh) {
    Serial.println(message);
  }
}/* end of threshold */




/* 
  Function to read the temperature and humidity values from the DHT 22 sensor module

  @temp : the temperature reading
  @hum : the humidity reading
  @return : returns 1 if reading is successfull or -1 if error occured
*/
int readSensor(float *temp, float *hum) {
  TempAndHumidity data = dht22.getTempAndHumidity();
  if (dht22.getStatus() == DHTesp::ERROR_NONE) {
      *temp = data.temperature;
      *hum  = data.humidity;
      return 1;
  } else {  return -1;  }
}/* end readSensor */



/* 
  Function that implements the linear squared approximation method

  @param a : the returned slope of the linear model
  @param b : the intercept of the linear model
  @param n : the number of inputs to apply the method 
*/
void lsq(float *a, float *b, const int n) {

  float SUMx  = .0; 
  float SUMy  = .0; 
  float SUMxy = .0; 
  float SUMxx = .0;

  // Here x[i] = i
  for( int i = 0; i < n; i++) {
    SUMx  += i;
    SUMy  += temp_vector[i];
    SUMxy += i * temp_vector[i];
    SUMxx += i * i;
  }
  
  *a = ( n * SUMxy - SUMx * SUMy) / ( SUMx*SUMx - n * SUMxx );
  *b = ( SUMy - (*a) * SUMx ) / n; 
}/* end of lsq */



/* 
  Function to append a new value to a circular buffer

  @val : the new value to append to the buffer
  @index : the current index of the buffer
  @n : the number of elements in the buffer
  @buff : the circular buffer
  @return : return the updated index
*/
int append(float val, int index, int n, float *buff) {
  
  // append the new measurement to the vector
  buff[index] = val;
  index = (index + 1) % n;

  #ifdef DEBUG_MODE
    Serial.print("[INFO] appendNewTemperature : Temperature Vector = {");
    for (int i = 0; i < n; i++) {
      Serial.print(buff[i]);
      if (i < n-1) {
        Serial.print(", ");
      }
    }
    Serial.println(" }");
  #endif

  return index;
}/* end append */




void setup() {
  Serial.begin(115200);
  while(!Serial){  delay(1000);  } // wait until serial is ready

  // initialize the sensor
  dht22.setup(DHT_PIN, DHTesp::DHT22);
  while (dht22.getStatus() != DHTesp::ERROR_NONE){  delay(1000);   }

  // Connect to WiFi
  wifiConnect(WIFI_SSID, WIFI_PASSWORD);

  // register callback function
  client.setCallback(callback);

  // form the mqtt topics
  sprintf(topicControl, "%s/%s", MQTT_USER, MQTT_TOPIC_CONTROL);
  sprintf(topicDataCrc, "%s/%s", MQTT_USER, MQTT_TOPIC_DATA_CRC);
  sprintf(topicData,    "%s/%s", MQTT_USER, MQTT_TOPIC_DATA);

  // Subscribe to topics and connect to MQTT server
  mqttConnect();
}



void loop() {

  // Reconnect if disconnected from WiFi
  if (WiFi.status() != WL_CONNECTED) {  
    unsigned long start_time = millis();

    wifiConnect(WIFI_SSID, WIFI_PASSWORD);  

    // if 1 min without connection reset control variables
    if(millis() - start_time > 60000){
      initialized = false;
      startMeasurements = false;
      lsqInit = false;
    }

  }



  // Interactions with the MQTT server if connected, else try to connect
  if ( client.connected() ) {
    // send message for configuration. Device not ready yet
    if( (!initialized) && (!startMeasurements) ) {
      client.publish(topicControl, "sendConfig");
    }

    // send CRC of the received configuration and wait for startMeasurements command
    if( (initialized) && (!startMeasurements) ) {
      client.publish(topicDataCrc, crc);
    }

  } else {   
    unsigned long start_time = millis();
    
    mqttConnect();  
    
    // if 1 min without connection reset control variables
    if(millis() - start_time > 60000){
      initialized = false;
      startMeasurements = false;
      lsqInit = false;
    }

  }



  // enable the lsq computation when the buffer is full with valid data.
  if ( (window_index == LSQ_WINDOW_SIZE-1) && (!lsqInit) && (startMeasurements) ){  lsqInit = true;  }



  // perform sampling and data processing
  if( (initialized) && (startMeasurements) ) {
    // Read and process data. Continue operating even if disconnected from server
    if ((millis() - measPreviousMillis) >=  sampling_time) {
    
      float temp = 0.0;
      float hum  = 0.0;
      float a    = 0.0;
      float b    = 0.0;

      // read sensor values
      if( readSensor(&temp, &hum) > 0 ) {
        
        // Threshold values
        threshold(temp, (float)temp_threshold, "[WARNING] Temperature High"); 
        threshold(hum, (float)hum_threshold, "[WARNING] Humidity High");
        
        // Display measured values
        Serial.println("[INFO] loop : Measurements");

        Serial.print("\tTemperature = ");
        Serial.print(temp);
        Serial.println(" *C");

        Serial.print("\tHumidity = ");
        Serial.print(hum);
        Serial.println(" %");

        // Append to cyclic buffer
        window_index = append(temp, window_index, LSQ_WINDOW_SIZE, temp_vector);

        // Compute the linear fit => rate of change : a
        if(lsqInit == true) {  
          lsq(&a, &b, LSQ_WINDOW_SIZE);  
       
          Serial.print("[INFO] loop : f(x) = ");
          Serial.print(a);
          Serial.print(" * x + ");
          Serial.println(b);
        }

      } else {  Serial.println("[ERROR] loop : Sensor is not working.");  }
          
      // update timer
      measPreviousMillis = millis();
    }
  }
  

  client.loop();
}



