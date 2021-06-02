#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <driver/gpio.h>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "./main.h"
#include "./.environment_variables.h"
#include "./wifi.h"
#include "./mqtt.h"

#if DEBUG_MODE
#define TAG "main.cpp"
#endif

extern "C" {
	void app_main(void);
}

void initNvsFlash() {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

enum StepperMotorPins {
    IN1 = GPIO_NUM_26,
    IN2 = GPIO_NUM_25,
    IN3 = GPIO_NUM_33,
    IN4 = GPIO_NUM_32,
    HOLD = 1,
    OFF = 0
};

StepperMotorPins getNextStepPin(StepperMotorPins currentPin, bool isClockwise) {
    switch (currentPin) {
        case StepperMotorPins::IN1: return isClockwise ? StepperMotorPins::IN2 : StepperMotorPins::IN4;
        case StepperMotorPins::IN2: return isClockwise ? StepperMotorPins::IN3 : StepperMotorPins::IN1;
        case StepperMotorPins::IN3: return isClockwise ? StepperMotorPins::IN4 : StepperMotorPins::IN2;
        case StepperMotorPins::IN4: return isClockwise ? StepperMotorPins::IN1 : StepperMotorPins::IN3;
    };
}

// Pin should be high if 1. pinToCompare is the pinToSet 2. if pinToCompare is set to HOLD
bool checkIfPinShouldBeHigh(StepperMotorPins pinToSet, StepperMotorPins pinToCompare) {
    return (pinToSet && pinToCompare) || pinToCompare == StepperMotorPins::HOLD;
}

// sets one of the four StepperMotorPins to high and the rest to low.
void setStepperMotorGPIOLevels(StepperMotorPins pin) {
    gpio_set_level((gpio_num_t)StepperMotorPins::IN1, 
        checkIfPinShouldBeHigh(StepperMotorPins::IN1,pin));
    gpio_set_level((gpio_num_t)StepperMotorPins::IN2, 
        checkIfPinShouldBeHigh(StepperMotorPins::IN2,pin));
    gpio_set_level((gpio_num_t)StepperMotorPins::IN3, 
        checkIfPinShouldBeHigh(StepperMotorPins::IN3,pin));
    gpio_set_level((gpio_num_t)StepperMotorPins::IN4, 
        checkIfPinShouldBeHigh(StepperMotorPins::IN4,pin));
}

int steps = 0;
bool isClockwise = true;
StepperMotorPins nextStepPin = StepperMotorPins::OFF;
void app_main()
{
    initNvsFlash();

    // config and start wifi
    registerWifiEventHandlers(); // this allocates the default event queue loop
    initWifi();
     
    // TODO: subscribe to mqtt task for controlling motors       
    while (1)
    {
        if (nextStepPin == StepperMotorPins::OFF || nextStepPin == StepperMotorPins::HOLD)
            vTaskDelay(500 / portTICK_RATE_MS);
        else {
            vTaskDelay(2/portTICK_RATE_MS); // delay 2 milliseconds before writing the next stepper motor command
            setStepperMotorGPIOLevels(nextStepPin);
        }
        if (steps > 1024) {
            isClockwise = !isClockwise;
            steps = 0;
        }
        nextStepPin = getNextStepPin(nextStepPin, isClockwise);
    }
}