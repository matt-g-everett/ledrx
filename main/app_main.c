#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include <stdio.h>

#include "mqtt_client.h"
#include "wifi.h"
#include "led.h"

#define STACK_SIZE 4096

static const char *TAG = "LEDRX";
static const char *ACK_TOPIC = "home/xmastree/ack";
static const char *LOG_TOPIC = "home/xmastree/log";
static const char *ACK_MSG_JSON = "{\"type\":\"ack\",\"ackID\":%u}";

static esp_mqtt_client_handle_t _mqtt_client;
static uint8_t _tasks_started = false;
static uint8_t _ackID = 0;

static void subscribe_led_stream(esp_mqtt_client_handle_t client, const char *advertise_topic) {
    int msg_id = esp_mqtt_client_subscribe(client, advertise_topic, 0);
    ESP_LOGI(TAG, "Sent subscribe to %s, msg_id=%d", advertise_topic, msg_id);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            ESP_LOGI(TAG, "***** ledrx started *****");

            // Hook-up LED stream
            subscribe_led_stream(event->client, CONFIG_LED_TOPIC_STREAM);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG, "MQTT_EVENT_DATA: topic=%.*s, data_len=%d",
                     event->topic_len, event->topic, event->data_len);
            if (event->topic_len > 0 && strncmp(event->topic, CONFIG_LED_TOPIC_STREAM, event->topic_len) == 0) {
                ESP_LOGD(TAG, "LED stream data received: %d bytes", event->data_len);
                led_push_stream(event->data);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

static esp_mqtt_client_handle_t mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .credentials.username = CONFIG_MQTT_USERNAME,
        .credentials.authentication.password = CONFIG_MQTT_PASSWORD
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    return client;
}

void start_tasks(void) {
    _mqtt_client = mqtt_app_start();

    // Pin LED task to Core 1 for dedicated LED processing
    // WiFi/MQTT tasks run on Core 0 by default
    xTaskCreatePinnedToCore(led_task, "led", STACK_SIZE, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "LED task pinned to Core 1");
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP sync");
    if (!_tasks_started) {
        start_tasks();
    }
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "0.uk.pool.ntp.org");
    esp_sntp_setservername(1, "1.uk.pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();

    // Set timezone to GMT
    setenv("TZ", "GMT0BST,M3.5.0/1,M10.5.0", 1);
    tzset();
}

static void led_ack_callback(uint8_t ackID)
{
    char message[40];

    // Handle the ack identifier, if it's not zero and it's changed, send a confirmation
    if (ackID != _ackID && ackID != 0) {
        sprintf(message, ACK_MSG_JSON, ackID);
        esp_mqtt_client_publish(_mqtt_client, ACK_TOPIC, message, 0, 0, 0);
    }
    _ackID = ackID;
}

static void log_callback(char *message)
{
    esp_mqtt_client_publish(_mqtt_client, LOG_TOPIC, message, 0, 0, 0);
}

void app_main()
{
    esp_err_t err;

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    wifi_init(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
    initialize_sntp();

    int gpios[2];
    gpios[0] = CONFIG_LED_GPIO_A;
    gpios[1] = CONFIG_LED_GPIO_B;
    led_initialise(log_callback, led_ack_callback, gpios, sizeof(gpios) / sizeof(int));
}
