#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <aquaman_config.h>

const char* SSID     = WIFI_SSID;
const char* wifiPassword = WIFI_PASSWORD;
const char* mqttServer = MQTT_SERVER_IP;
const uint16_t mqttPort = MQTT_PORT;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

StaticJsonDocument<200> connectJSON;
char jsonBuffer[1024];
String mqttClientId;
bool serverConnected = false;

long lastReportTimeStamp = 0;

void connectWifi() {
  delay(10);
    Serial.println();
  Serial.print("Connecting to WiFi network: ");
  Serial.println(SSID);

  WiFi.begin(SSID, wifiPassword);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  }
}

void createConnectMessage() {
  connectJSON["deviceId"] = AQUAMAN_CLIENT_ID;
  connectJSON["slotId"] = "none";
  connectJSON["timestamp"] = "none";
  connectJSON["payload"] = "none";
}

void createMQTTclientId() {
  mqttClientId = "AquamanClient-";
  mqttClientId += String(AQUAMAN_CLIENT_ID);
  mqttClientId += String("-");
  mqttClientId += String(random(0xffff), HEX);
}

void initMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
  createConnectMessage();
  createMQTTclientId();
}

void sendConnectMessage() {
  Serial.println("Connecting to Aquaman server...");
  serializeJson(connectJSON, jsonBuffer);
  mqttClient.publish(MQTT_TOPIC_OUT, jsonBuffer);
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to mqtt://");
    Serial.print(mqttServer);
    Serial.print(":");
    Serial.println(mqttPort);
    
    Serial.print("MQTT client id: ");
    Serial.println(mqttClientId);
    if (mqttClient.connect(mqttClientId.c_str())) {
      Serial.println("connected to MQTT :)");
      sendConnectMessage();
      mqttClient.subscribe(MQTT_TOPIC_IN);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  connectWifi();
  randomSeed(micros());
  initMQTT();
  connectMQTT();
}

void loop() {

  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  long now = millis();
  if ((now - lastReportTimeStamp > REPORTING_FEQUENCY) && (!serverConnected)) {
    lastReportTimeStamp = now;
    sendConnectMessage();
  }
}