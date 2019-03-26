#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Task.h>
#include "ButtonTask.h"
#include <aquaman_config.h>
#include <secrets.h>

const char* SSID     = WIFI_SSID;
const char* wifiPassword = WIFI_PASSWORD;

const char* mqttServer = MQTT_SERVER_IP;
const uint16_t mqttPort = MQTT_PORT;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

TaskManager taskManager;

void handleReportingLoop(uint32_t deltaTime);
void handleLEDinit(uint32_t deltaTime);
void handleFilterSwitch(ButtonState state);
void handleLightSwitch(ButtonState state);
void handleCO2Switch(ButtonState state);

FunctionTask taskLEDInit(handleLEDinit, MsToTaskTime(200));
FunctionTask taskReportingLoop(handleReportingLoop, MsToTaskTime(REPORTING_FEQUENCY));
ButtonTask taskFilterSwitch(handleFilterSwitch, SWITCH_GPIO_FILTER);
ButtonTask taskLightSwitch(handleLightSwitch, SWITCH_GPIO_LIGHT);
ButtonTask taskCO2Switch(handleCO2Switch, SWITCH_GPIO_CO2);

StaticJsonDocument<JSON_BUFFER_SIZE> connectJSON;
StaticJsonDocument<JSON_BUFFER_SIZE> statusJSON;
StaticJsonDocument<JSON_BUFFER_SIZE> incomingJSON;
StaticJsonDocument<JSON_BUFFER_SIZE> stateToggleRequestJSON;
char jsonBuffer[JSON_BUFFER_SIZE];

char mqttClientId[64];
bool serverConnected = false;

unsigned long lastReportTimeStamp = 0;
unsigned long syncTimestamp;
unsigned long deltaTimestamp;

struct tSlots {
  char id[16];
  uint8_t relayGPIO;
  uint8_t switchGPIO;
  uint8_t ledGPIO;
  bool state;
};
tSlots slots[SLOTS_COUNT];

unsigned long getCurrentTimestamp() {
  return syncTimestamp + (millis() / 1000 - deltaTimestamp);
}

void createStatusMessage() {
  statusJSON["deviceId"] = AQUAMAN_CLIENT_ID;
  statusJSON["timestamp"] = getCurrentTimestamp();
  statusJSON["command"] = COMMAND_REPORT_STATUS;
  for (int i = 0; i < SLOTS_COUNT; i++) {
    statusJSON["payload"]["slots"][slots[i].id] = slots[i].state ? STATE_ON : STATE_OFF;
  }
}

void sendStatusMessage() {
  Serial.println("[INFO]  sending status update");
  createStatusMessage();
  serializeJson(statusJSON, jsonBuffer);
  mqttClient.publish(MQTT_TOPIC_OUT, jsonBuffer);
}

/**
 * @brief Sets the slot ids (filter, light etc...), and assigns the GPIO ids to them, also
 * sets up the relay slot pins and sets them to low state
 */
void initSlots() {
  strncpy(slots[0].id, SLOT_ID_FILTER, 16);
  slots[SLOT_INDEX_FILTER].relayGPIO = RELAY_GPIO_FILTER;
  slots[SLOT_INDEX_FILTER].switchGPIO = SWITCH_GPIO_FILTER;
  slots[SLOT_INDEX_FILTER].ledGPIO = LED_GPIO_FILTER;
  strncpy(slots[1].id, SLOT_ID_LIGHT, 16);
  slots[SLOT_INDEX_LIGHT].relayGPIO = RELAY_GPIO_LIGHT;
  slots[SLOT_INDEX_LIGHT].switchGPIO = SWITCH_GPIO_LIGHT;
  slots[SLOT_INDEX_LIGHT].ledGPIO = LED_GPIO_LIGHT;
  strncpy(slots[2].id, SLOT_ID_CO2, 16);
  slots[SLOT_INDEX_CO2].relayGPIO = RELAY_GPIO_CO2;
  slots[SLOT_INDEX_CO2].switchGPIO = SWITCH_GPIO_CO2;
  slots[SLOT_INDEX_CO2].ledGPIO = LED_GPIO_CO2;
  for (int i = 0; i < SLOTS_COUNT; i++) {
    slots[i].state = false;
    pinMode(slots[i].relayGPIO, OUTPUT);
    pinMode(slots[i].ledGPIO, OUTPUT);
    pinMode(slots[i].switchGPIO, INPUT_PULLUP);
    digitalWrite(slots[i].ledGPIO, LOW);
    digitalWrite(slots[i].relayGPIO, HIGH);
    delay(10);
  }
  Serial.println("[INFO]  slots initialized");
}

int getSlotIndex(const char* slotId) {
  int slotIndex = 0;
  while ((slotIndex < SLOTS_COUNT) && (strcmp(slots[slotIndex].id, slotId))) { slotIndex++; }
  return (slotIndex < SLOTS_COUNT) ? slotIndex : -1;
}

/**
 * @brief Sets the state of one slot
 * 
 * @param slotIndex - the numeric state of the slot
 * @param booleanState - the new state represented as boolean
 */
void setSlotState(unsigned int slotIndex, bool booleanState) {
  Serial.printf("[INFO]  setting slot (%s) state to \"%s\"\n", slots[slotIndex].id, booleanState ? STATE_ON : STATE_OFF);
  if (slots[slotIndex].state == booleanState) {
    return;
  }
  
  slots[slotIndex].state = booleanState;
  if (booleanState) {
      digitalWrite(slots[slotIndex].ledGPIO, HIGH);
      digitalWrite(slots[slotIndex].relayGPIO, LOW);
  } else {
      digitalWrite(slots[slotIndex].ledGPIO, LOW);
      digitalWrite(slots[slotIndex].relayGPIO, HIGH);
  }

  sendStatusMessage();
}

void requestStateToggle(int slotIndex) {
  Serial.println("[INFO]  sending state toggle request");
  stateToggleRequestJSON["deviceId"] = AQUAMAN_CLIENT_ID;
  stateToggleRequestJSON["timestamp"] = getCurrentTimestamp();
  stateToggleRequestJSON["command"] = COMMAND_TOGGLE_REQUEST;
  stateToggleRequestJSON["payload"]["slotId"] = slots[slotIndex].id;
  stateToggleRequestJSON["payload"]["state"] = !slots[slotIndex].state;
  serializeJson(stateToggleRequestJSON, jsonBuffer);
  mqttClient.publish(MQTT_TOPIC_OUT, jsonBuffer);
}

void toggleSlotState(int slotIndex) {
  setSlotState(slotIndex, !slots[slotIndex].state);
}

void handleFilterSwitch(ButtonState state) {
  if (state == ButtonState_Pressed) {
    if (serverConnected) {
      requestStateToggle(SLOT_INDEX_FILTER);
    } else {
      toggleSlotState(SLOT_INDEX_FILTER);
    }
  }
}

void handleLightSwitch(ButtonState state) {
  if (state == ButtonState_Pressed) {
    if (serverConnected) {
      requestStateToggle(SLOT_INDEX_LIGHT);
    } else {
      toggleSlotState(SLOT_INDEX_LIGHT);
    }
  }
}

void handleCO2Switch(ButtonState state) {
  if (state == ButtonState_Pressed) {
    if (serverConnected) {
      requestStateToggle(SLOT_INDEX_CO2);
    } else {
      toggleSlotState(SLOT_INDEX_CO2);
    }
  }
}

void connectWifi() {
  delay(10);
  Serial.printf("\n[INFO]  Connecting to WiFi network: %s\n", SSID);

  WiFi.begin(SSID, wifiPassword);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("[INFO]  WiFi connected, ip: ");
  Serial.println(WiFi.localIP());
}

/**
 * @brief Sets the current timestamp to the given parameter
 * 
 * @param timestamp - current timestamp
 */
void syncTime(unsigned long timestamp) {
  syncTimestamp = timestamp;
  deltaTimestamp = millis() / 1000;
  Serial.printf("[INFO]  time sync completed, timestamp set to: %lu\n", syncTimestamp);
}

/**
 * @brief MQTT message callback function
 * 
 * @param topic 
 * @param message 
 * @param length 
 */
void onMessage(char* topic, byte* message, unsigned int length) {
  memset(jsonBuffer, 0, JSON_BUFFER_SIZE);
  for (unsigned int i = 0; i < length; i++) {
    jsonBuffer[i] = (char)message[i];
  }
  DeserializationError error = deserializeJson(incomingJSON, jsonBuffer);

  if (error) {
    Serial.printf("[ERROR] JSON deserialization failed! (%s)", error.c_str());
    return;
  }
  const char* deviceId = incomingJSON["deviceId"];

  if (strcmp(deviceId, AQUAMAN_CLIENT_ID) == 0) {
    serverConnected = true;
    const char* command = incomingJSON["command"];
    Serial.printf("[INFO]  command received: \"%s\"\n", command);
    if (strcmp(command, COMMAND_SYNC_TIME) == 0) {
      unsigned long timestamp = incomingJSON["timestamp"];
      syncTime(timestamp);
    } else if (strcmp(command, COMMAND_SET_STATUS) == 0) {
      const char* slotId = incomingJSON["payload"]["slotId"];
      int slotIndex = getSlotIndex(slotId);
      if (slotIndex == -1) {
        Serial.printf("[ERROR] unknown slot id received: \"%s\"\n", slotId);
      } else {
        const char* state = incomingJSON["payload"]["state"];
        if (strcmp(state, STATE_ON) == 0) {
          setSlotState(slotIndex, true);
        } else if (strcmp(state, STATE_OFF) == 0) {
          setSlotState(slotIndex, false);
        } else {
          Serial.printf("[ERROR] unknown state received: \"%s\"\n", state);
        }
      }
    } else {
      Serial.printf("[ERROR] unknown command received: \"%s\"\n", command);
    }
  }
}

void createConnectMessage() {
  connectJSON["deviceId"] = AQUAMAN_CLIENT_ID;
  connectJSON["timestamp"] = "none";
  connectJSON["command"] = COMMAND_REQUEST_SYNC_TIME;
}                  

void createMQTTclientId() {
  sprintf(mqttClientId, "AquamanClient-%s-%04x", AQUAMAN_CLIENT_ID, random(0xffff));
  Serial.printf("[INFO]  MQTT client id: %s\n", mqttClientId);
}

void initMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(onMessage);
  createConnectMessage();
  createMQTTclientId();
}

void sendConnectMessage() {
  Serial.println("[INFO]  sending time sync request");
  serializeJson(connectJSON, jsonBuffer);
  mqttClient.publish(MQTT_TOPIC_OUT, jsonBuffer);
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("[INFO]  connecting to MQTT");
    if (mqttClient.connect(mqttClientId)) {
      Serial.println("[INFO]  connected to MQTT");
      sendConnectMessage();
      mqttClient.subscribe(MQTT_TOPIC_IN);
    } else {
      Serial.printf("[ERROR] MQTT connection failed, rc=%d (retrying in 5 secs)\n", mqttClient.state());
      delay(5000);
    }
  }
}

void handleReportingLoop(uint32_t deltaTime) {
  if (!mqttClient.connected()) {
    serverConnected = false;
    connectMQTT();
  } else if (!serverConnected) {
    sendConnectMessage();
  } else {
    sendStatusMessage();
  }
}

void handleLEDinit(uint32_t delta_time) {
  if (serverConnected) {
    return;
  }
  for (int i = 0; i < SLOTS_COUNT; i++) {
    digitalWrite(slots[i].ledGPIO, HIGH);
    delay(20);
    digitalWrite(slots[i].ledGPIO, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println();
  Serial.printf("-------- Starting Aquaman client (id: \"%s\") --------\n", AQUAMAN_CLIENT_ID);
  connectWifi();
  randomSeed(micros());
  initMQTT();
  initSlots();
  taskManager.StartTask(&taskLEDInit);
  taskManager.StartTask(&taskReportingLoop);
  taskManager.StartTask(&taskFilterSwitch);
  taskManager.StartTask(&taskLightSwitch);
  taskManager.StartTask(&taskCO2Switch);
}

void loop() {
  taskManager.Loop();
  mqttClient.loop();
}