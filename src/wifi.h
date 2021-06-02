#ifndef wifi_H
#define wifi_H

#include "esp_wifi.h" //espidf module

#include "./.environment_variables.h"

#if DEBUG_MODE
#define TAG "wifi_H"
#endif

static EventGroupHandle_t wifiEventGroup;

enum wifiEvents {
    WIFI_CONNECTING_GROUPEVENT_FLAG = BIT0,
    WIFI_CONNECTED_GROUPEVENT_FLAG = BIT1,
    WIFI_DISCONNECTED_GROUPEVENT_FLAG = BIT2,
    WIFI_ERROR_GROUPEVENT_FLAG = BIT3
};

// when the wifi station is started and is ready, set the GROUPEVENT flag and try to connect to AP
void wifiConnect() {
    #if DEBUG_MODE
        ESP_LOGI(TAG, "Connecting to %s with password %s", wifiSsid, wifiPassword);
    #endif
    xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTING_GROUPEVENT_FLAG);
    esp_wifi_connect();
}
// Set the EVENTGROUP flag to disconnect and try to reconnect after 5 seconds
void onWifiDisconnected() {
    #if DEBUG_MODE
        ESP_LOGI(TAG, "Disconnected from %s. Reconnecting in 5 seconds", wifiSsid);
    #endif
    xEventGroupSetBits(wifiEventGroup, WIFI_DISCONNECTED_GROUPEVENT_FLAG);
    vTaskDelay(5000 / portTICK_RATE_MS);
    wifiConnect();
}
// set the EVENTGROUP flag to connected
void onWifiGotIP(void* event_data) {
    #if DEBUG_MODE
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    #endif
    xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_GROUPEVENT_FLAG);
}

// based on https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/station/main/station_example_main.c
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        wifiConnect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        onWifiDisconnected();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_AUTHMODE_CHANGE) {        
        xEventGroupSetBits(wifiEventGroup, WIFI_ERROR_GROUPEVENT_FLAG);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        onWifiGotIP(event_data);
    }
    #if DEBUG_MODE
        else {
            if (event_base == WIFI_EVENT)
                ESP_LOGW(TAG, "Received unhandled WIFI Event %d ", event_id);
            if (event_base == IP_EVENT)
                ESP_LOGW(TAG, "Received unhandled IP Event %d ", event_id);
        }
    #endif
}

// register event handlers for WIFI events and GOT IP event
void registerWifiEventHandlers() {
    esp_event_loop_create_default();
    wifiEventGroup = xEventGroupCreate();

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id);

    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip);
}

void initWifi() {
    esp_netif_init();
    // tcpip_adapter_init(); // https://github.com/espressif/esp-adf/issues/564

    esp_netif_create_default_wifi_sta();

    #define CONFIG_ESP32_WIFI_CACHE_TX_BUFFER_NUM 32
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    

    esp_wifi_set_mode(WIFI_MODE_STA);

    wifi_sta_config_t sta = {wifiSsid, wifiPassword};
    sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    sta.pmf_cfg = {true, false};
    wifi_config_t wifi_config = {.sta = sta};
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    
    esp_wifi_start();
}

#endif