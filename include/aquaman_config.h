#define MQTT_SERVER_IP "192.168.178.116"
#define MQTT_PORT 1883
#define AQUAMAN_CLIENT_ID "45P"
#define MQTT_TOPIC_IN "control"
#define MQTT_TOPIC_OUT "report"
#define REPORTING_FEQUENCY 5000
#define COMMAND_SYNC_TIME "sync_time"
#define COMMAND_REQUEST_SYNC_TIME "req_sync_time"
#define COMMAND_SET_STATUS "set_status"
#define COMMAND_REPORT_STATUS "report_status"
#define COMMAND_TOGGLE_REQUEST "toggle_request"
#define JSON_BUFFER_SIZE 1024
#define SLOTS_COUNT 3

#define SLOT_INDEX_FILTER 0
#define SLOT_ID_FILTER "filter"
#define RELAY_GPIO_FILTER 16
#define SWITCH_GPIO_FILTER 0
#define LED_GPIO_FILTER 12

#define SLOT_INDEX_LIGHT 1
#define SLOT_ID_LIGHT "light"
#define RELAY_GPIO_LIGHT 5
#define SWITCH_GPIO_LIGHT 2
#define LED_GPIO_LIGHT 13

#define SLOT_INDEX_CO2 2
#define SLOT_ID_CO2 "co2"
#define RELAY_GPIO_CO2 4
#define SWITCH_GPIO_CO2 14
#define LED_GPIO_CO2 15

#define STATE_ON "on"
#define STATE_OFF "off"
