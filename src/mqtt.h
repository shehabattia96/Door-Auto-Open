#ifndef mqtt_H
#define mqtt_H

#include "mqtt_client.h" //espidf module

#include "./.environment_variables.h"

#if DEBUG_MODE
#define TAG "mqtt"
#endif

esp_mqtt_client_handle_t client;

void startMQTTClient()
{
    esp_mqtt_client_config_t mqtt_cfg = {
      .host = mqttHost,
      .port = mqttPort,
      .client_id = mqttClientId
    };
    
    client = esp_mqtt_client_init(&mqtt_cfg);
   
    esp_mqtt_client_start(client);
}

#endif