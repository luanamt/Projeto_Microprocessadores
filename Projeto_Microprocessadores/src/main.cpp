/*#include <Arduino.h>
#include <WiFi.h>


const char* ssid = "VIVOFIBRA-AE86";
const char* password =  "M3lz1n34@Lun1n34@B1lul1n34)$(";
 
void setup() {
 Serial.begin(115200);
 WiFi.begin(ssid, password);
 
 while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.println("Connecting to WiFi..");
 }
 Serial.println("Connected to the WiFi network");
}
 
void loop() {}

/*void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
}
 //--------------------------------------------------------

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Serial connected 250Kbps...");
  float sum=0;
  for(int i=0;i<100;i++){
    sum += analogRead(A0);
  }
  Serial.print("ADC Value: ");
  Serial.print(((sum/102300)*3.3*1000)-32); //Precisamos criar uma fórmula para fazer a linearização da tempera, ou seja, calibrar o sensor.
  Serial.print(" mV.");
  Serial.print(" - Temperature: ");
  Serial.print((((sum/102300)*3.3*1000)-32)/10);
  Serial.print(" oC.\n");
  delay(500);
}
*/


/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/

#include <Arduino.h>
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"

#define MQTT_HOST IPAddress(192, 168, 15, 223)
#define MQTT_PORT 1883
#define MQTT_TOPIC "test"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  
  uint16_t packetIdSub0 = mqttClient.subscribe("LED_R", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub0);
  uint16_t packetIdSub1 = mqttClient.subscribe("LED_G", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub1);
  uint16_t packetIdSub2 = mqttClient.subscribe("LED_B", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub2);
  //
  //mqttClient.publish(MQTT_TOPIC, 0, true, "test 1");
  //Serial.println("Publishing at QoS 0");
  //
  //uint16_t packetIdPub1 = mqttClient.publish(MQTT_TOPIC, 1, true, "test 2");
  //Serial.print("Publishing at QoS 1, packetId: ");
  //Serial.println(packetIdPub1);
  //
  //uint16_t packetIdPub2 = mqttClient.publish(MQTT_TOPIC, 2, true, "test 3");
  //Serial.print("Publishing at QoS 2, packetId: ");
  //Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  //Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttPublish(uint16_t packetId) {
  //Serial.println("Publish acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

void onMqttUnsubscribe(uint16_t packetId) {
  //Serial.println("Unsubscribe acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

// setting PWM properties
#define ledPin_R 16
#define ledPin_G 17
#define ledPin_B 5
#define ledChannel_R 0
#define ledChannel_G 1
#define ledChannel_B 2
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8 //Resolution 8, 10, 12, 15

void setLedRed(uint8_t payload_int) {
  Serial.print("setLedRed : ");
  Serial.println(payload_int);
  ledcWrite(ledChannel_R, (255-payload_int));
};

void setLedGreen(uint8_t payload_int) {
  Serial.print("setLedGreen : ");
  Serial.println(payload_int);
  ledcWrite(ledChannel_G, (255-payload_int));
};

void setLedBlue(uint8_t payload_int) {
  Serial.print("setLedBlue : ");
  Serial.println(payload_int);
  ledcWrite(ledChannel_B, (255-payload_int));
};

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String payload_string = payload;
  payload_string.substring(0,len);
  int payload_int = payload_string.toInt();
  
  String topic_string = topic;


  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
  Serial.print("  Payload: ");
  Serial.println(payload_int);
  Serial.println("----------------");
  Serial.println(payload);
  Serial.println("----------------");

  if (topic_string.equals("LED_R")) {
      setLedRed(payload_int);
  }
  if (topic_string.equals("LED_G")) {
      setLedGreen(payload_int);
  }
  if (topic_string.equals("LED_B")) {
      setLedBlue(payload_int);
  }
}


void setup() {
  Serial.begin(250000);
  Serial.println();
  Serial.println();

  //set ADC1_CH0 pin as input:
  pinMode(36, INPUT);
  //set ADC input attenuation:
  //analogSetPinAttenuation(36, (adc_attenuation_t)ADC_0db);// specific pin
  analogSetAttenuation((adc_attenuation_t)ADC_0db); // all ADC pins
  // Set the divider for the ADC clock, default is 1, range is 1 - 255
  analogSetClockDiv(255); // 1338mS

  // Set RGB LED pins as outputs with open drain:
  pinMode(ledPin_R, OUTPUT_OPEN_DRAIN);
  pinMode(ledPin_R, OUTPUT_OPEN_DRAIN);
  pinMode(ledPin_R, OUTPUT_OPEN_DRAIN);
  // attach the channel to be controlled:
  ledcAttachPin(ledPin_R, ledChannel_R);
  ledcAttachPin(ledPin_G, ledChannel_G);
  ledcAttachPin(ledPin_B, ledChannel_B);
  // configure LED PWM functionalitites
  ledcSetup(ledChannel_R, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(ledChannel_G, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(ledChannel_B, PWM_FREQ, PWM_RESOLUTION);

  // turn off LED
  ledcWrite(ledChannel_R, 255);
  ledcWrite(ledChannel_G, 255);
  ledcWrite(ledChannel_B, 255);


  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}



/*******************************************************
Sensor: LM35
Sensor caracteristics:
Sensor output is linear: Vout = 0.0mV + 10.0mV/°C
Sensor output voltage table:
LM35 at   0°C =   0.0 mV
LM35 at 100°C = 1000.0mV
*******************************************************/

/*******************************************************
Sensor: LDR
Sensor caracteristics:
LDR in day light                   =  300 Ohm
LDR in a room light with lights on = 1750 Ohm
LDR in the dark                    =    2 MOhm

LDR Input conditining circuit:
Voltage divider 4K7 + 2K2
Input voltage : 3300mV
Output voltage: 1052mV

LDR voltage divider:
LDR + 1Kohm
Input voltage : 1052mV

Sensor output voltage:
LDR in day light                   : Vout =  242.8mV
LDR in a room light with lights on : Vout =  669.5mV
LDR in the dark                    : Vout = 1051.6mV

*******************************************************/

/*******************************************************
ADC caracteristics:
ADC attenuation set to: ADC_0db (ADC reference = 1100mV)

ADC has a dead band:
input   50mV : ADC reads    0
input 1050mV : ADC reads 4095
*******************************************************/

int  lastMillis;
char sensor_text[100];
#define ADC_SAMPLES 10
#define ADC_CONST 0.0256410256410256410256410  //(1/ADC_SAMPLES)*(1050/4095)
#define ADC_LINEAR_CONST 50
int    adc_sum = 0;
float  adc_avg;
float  LM35_temp_c;
float  LDR_light;
int    i = 0;
bool   last_loop_finished = true;


void loop() {
   
   if ((millis() - lastMillis > 1000) && last_loop_finished) {
      last_loop_finished = false;
      lastMillis = millis();

      //Serial.println("Reading LM35 sensor...");
      adc_sum = 0;
      for(i=0; i<ADC_SAMPLES; i++) {
        adc_sum += analogRead(36);
        delay(1);
      }
      adc_avg = (((float)adc_sum)*ADC_CONST)+ADC_LINEAR_CONST;
      LM35_temp_c = (float)adc_avg/10;
      //sprintf(sensor_text, "sensor voltage: %f, LM35 temperature in °C: %f", adc_avg, LM35_temp_c);
      sprintf(sensor_text, "%f", LM35_temp_c);
      mqttClient.publish("temperature_c", 0, true, sensor_text);


      //Serial.println("Reading LDR sensor...");
      adc_sum = 0;
      for(i=0; i<ADC_SAMPLES; i++) {
        adc_sum += analogRead(39);
        delay(1);
      }
      //adc_avg = (((float)adc_sum)*ADC_CONST)+ADC_LINEAR_CONST;
      adc_avg = ((float)adc_sum)/409.5;
      //LDR_light = adc_avg;
      LDR_light = 100-adc_avg;
      sprintf(sensor_text, "%f", LDR_light);
      mqttClient.publish("light", 0, true, sensor_text);

      last_loop_finished = true;
  }
}