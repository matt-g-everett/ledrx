#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"

#include "mqtt_client.h"
#include "iotp_ota.h"
#include "iotp_wifi.h"
#include "led.h"

#define STACK_SIZE 4096

static const char *TAG = "LEDRX";
static const char *SOFTWARE = "ledrx";

// Embedded files
extern const uint8_t version_start[] asm("_binary_version_txt_start");
extern const uint8_t version_end[] asm("_binary_version_txt_end");

static mqtt_ota_state_handle_t _mqtt_ota_state;
static uint8_t _tasks_started = false;

static void subscribe_led_stream(esp_mqtt_client_handle_t client, const char *advertise_topic) {
    int msg_id = esp_mqtt_client_subscribe(client, advertise_topic, 0);
    ESP_LOGI(TAG, "Sent subscribe to %s, msg_id=%d", advertise_topic, msg_id);
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            ESP_LOGI(TAG, "***** ledrx started, version: %s *****", (const char *)version_start);

            // Hook-up OTA
            mqtt_ota_set_connected(_mqtt_ota_state, true);
            mqtt_ota_subscribe(event->client, CONFIG_OTA_TOPIC_ADVERTISE);

            // Hook-up LED stream
            subscribe_led_stream(event->client, CONFIG_LED_TOPIC_STREAM);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_ota_set_connected(_mqtt_ota_state, false);
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
            if (event->topic_len > 0 && strncmp(event->topic, CONFIG_LED_TOPIC_STREAM, event->topic_len) == 0) {
                led_push_stream(event->data);
            }
            else {
                mqtt_ota_handle_data(_mqtt_ota_state, event, CONFIG_OTA_TOPIC_ADVERTISE);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static esp_mqtt_client_handle_t mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .event_handle = mqtt_event_handler,
        .username = CONFIG_MQTT_USERNAME,
        .password = CONFIG_MQTT_PASSWORD
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    return client;
}

static void handle_ota_state_change(uint8_t started) {
    led_set_running(!started);
}

void start_tasks(void) {
    esp_mqtt_client_handle_t client = mqtt_app_start();
    _mqtt_ota_state = mqtt_ota_init(client, SOFTWARE, (const char *)version_start, handle_ota_state_change);
    xTaskCreate(mqtt_ota_task, "ota", STACK_SIZE, _mqtt_ota_state, 5, NULL);

    xTaskCreate(led_task, "led", STACK_SIZE, NULL, 5, NULL);
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
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "0.uk.pool.ntp.org");
    sntp_setservername(1, "1.uk.pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

    // Set timezone to GMT
    setenv("TZ", "GMT0BST,M3.5.0/1,M10.5.0", 1);
    tzset();
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
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    wifi_init(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
    initialize_sntp();
    led_initialise(CONFIG_LED_GPIO);
}
