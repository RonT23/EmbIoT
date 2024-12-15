#include <WiFi.h>
#include <DHTesp.h> 
#include <PubSubClient.h>

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

const uint8_t GENERATOR = 10; // CRC generator

const int LSQ_WINDOW_SIZE = 5;

int sampling_time  = 1000; // sampling period
int temp_threshold = 0;    // temperature threshold
int hum_threshold  = 0;    // humidity threshold

float temp_vector[LSQ_WINDOW_SIZE] = {0}; // cyclic buffer
int window_index                   = 0;

unsigned long measPreviousMillis = 0;

char crc[10];
bool initialized = false;
bool startMeasurements = false;

void callback(char*, byte*, unsigned int);

WiFiClient wifiClient;
PubSubClient client(MQTT_ADDRESS, 1883, callback, wifiClient);

const int DHT_PIN  = 23;
DHTesp dht22;

// 8-bit CRC basic function
uint8_t crc8_bit(uint8_t byte, const char generator) {
  uint8_t crc = byte;     

  for(int i = 0; i < 8; i++) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ generator;
    } else {
      crc <<= 1;
    }
  }

  return crc;
}

// Encode a 6-digit number
void encode_crc8_bit(const char* input, uint8_t generator, char* output) {

    char buffer[3];
    int outputIndex = 0;

    for (int i = 0; i < 3; i++) {
        strncpy(buffer, &input[i * 2], 2);
        buffer[2] = '\0'; // Null-terminate the string

        // Convert the 2-digit string to an integer
        uint8_t byteVal = (uint8_t)atoi(buffer);

        // Compute the CRC-8
        uint8_t crc = crc8_bit(byteVal, generator);

        // Format the result as a 3-digit string and append to output
        sprintf(&output[outputIndex], "%03d", crc);
        outputIndex += 3;
    }

    output[9] = '\0'; // Null-terminate the final encoded string
}

// MQTT Callback function
void callback(char* topic, byte* payload, unsigned int length) {
 
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;

  String message(p);
  String topic_str(topic);

  // Topic Data Callbacks
  if (topic_str == topicData) {
    
    // Encode the configuration parameters with 8-bit CRC
    encode_crc8_bit(message.c_str(), GENERATOR, crc);
    Serial.print("[INFO] Calculated CRC: ");
    Serial.println(crc);

    // Get the configurations parameters
    String DDstr = message.substring(0, 2);
    String TTstr = message.substring(2, 4);
    String HHstr = message.substring(4, 6);

    sampling_time  = DDstr.toInt() * 1000;
    temp_threshold = TTstr.toInt();
    hum_threshold  = HHstr.toInt();

    Serial.println("[INFO] callback : Configuration parameters ");
    Serial.print("\tSampling Time (s) : ");
    Serial.println(sampling_time);

    Serial.print("\tTemperature Threshold (C) : ");
    Serial.println(temp_threshold);

    Serial.print("\tHumidity Threshold (%) : ");
    Serial.println(hum_threshold);

    // Update flag into initialized
    initialized = true;
  }
  
  // Topic Data CRC callbacks
  if (topic_str == topicDataCrc) {
    if (message == "crcError") {
      Serial.println("[ERROR] callback: crcError");
      initialized = false;
    }
  }
  
  // Topic Control callbacks
  if (topic_str == topicControl) {
    if (message == "updateConfig") {
      Serial.println("[STATUS] callback: updateConfig");
      initialized = false;
      startMeasurements = false;

    } else if (message == "startMeasurements") {
      Serial.println("[STATUS] callback: startMeasurements");
      startMeasurements = true;
    }
  } 

}

// MQTT connect
void mqttConnect() {
  
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("[STATUS] mqttConnect : Attempting MQTT connection...");
    
    // Attempt to connect
    if (client.connect(MQTT_CLIENT, "samos", "karlovasi33#")) {
      Serial.println("[INFO] mqttConnect : MQTT connected");
      topicSubscribe();
    } 
    
    else {
      Serial.print("[ERROR] mqttConnect : rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }

  }

}

// MQTT topic subscribe
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
}

// WiFi connect
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
}

// Threshold and display message if val > threshold
void threshold(float val, float thresh, const char* message) {
  if (val > thresh) {
    Serial.println(message);
  }
}

// Read temperature and humidity from DHT 22 sensor
int readSensor(float *temp, float *hum) {
  TempAndHumidity data = dht22.getTempAndHumidity();

  if (dht22.getStatus() == DHTesp::ERROR_NONE) {
      *temp = data.temperature;
      *hum  = data.humidity;
      return 1;
  } else {  return -1;  }
}

// Compute the rate of change of temperature using the least squares method
void lsq(float *a, float *b, const int n) {
  float x[n] = {0};
  for (int i = 0; i < n; i++){
    x[i] = i;
  }

  float SUMx  = .0; 
  float SUMy  = .0; 
  float SUMxy = .0; 
  float SUMxx = .0;

  for( int i = 0; i < n; i++) {
    SUMx  += x[i];
    SUMy  += temp_vector[i];
    SUMxy += x[i] * temp_vector[i];
    SUMxx += x[i] * x[i];
  }
  
  // compute the rate of change and intercept
  *a = ( SUMx * SUMy - n * SUMxy ) / ( SUMx*SUMx - n * SUMxx );
  *b = ( SUMy - (*a) * SUMx ) / n; 

  Serial.print("[INFO] lsq : f(x) = ");
  Serial.print(*a);
  Serial.print(" * x + ");
  Serial.println(*b);
}

void appendNewTemperature(float new_temp, float *a, float *b) {
  temp_vector[window_index] = new_temp;
  window_index = (window_index + 1) % LSQ_WINDOW_SIZE;
  if(index == 0) {
    lsq(a, b, LSQ_WINDOW_SIZE);
  }
}

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
  } else {   mqttConnect();  }

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
        // Display measured values
        Serial.println("[INFO] loop : Measurements");

        Serial.print("\tTemperature = ");
        Serial.print(temp);
        Serial.println(" *C");

        Serial.print("\tHumidity = ");
        Serial.print(hum);
        Serial.println(" %");
        
        // Threshold values
        threshold(temp, (float)temp_threshold, "[WARNING] Temperature High"); 
        threshold(hum, (float)hum_threshold, "[WARNING] Humidity High");
          
        // Compute temperatures rate of change
        appendNewTemperature(temp, &a, &b);
      } else {  Serial.println("[ERROR] loop : Sensor is not working.");  }
          
      // update timer
      measPreviousMillis = millis();
    }
  }
  
  // Reconnect if disconnected from WiFi
  if (WiFi.status() != WL_CONNECTED) {  wifiConnect(WIFI_SSID, WIFI_PASSWORD);  }

  client.loop();
}



