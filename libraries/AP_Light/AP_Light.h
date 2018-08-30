#pragma once

#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include <GCS_MAVLink/GCS_MAVLink.h>

#define WS2812B_NUMBER                  8

// struct Ws2812bCtx {
//     mutex_t mtx;
//     uint32_t fb[WS2812B_NUMBER];

//     stm32_gpio_t *port;
//     uint16_t pin;
//     uint16_t MASK;
//     uint16_t ctl[WS2812B_NUMBER * 24];
//     uint8_t demo;
// } ledCtx;

class AP_Light
{
    friend class RGBLed;            // RGBLed needs access to Light parameters
    friend class Display;           // Display needs access to Light parameters
public:
    AP_Light();

    /* Do not allow copies */
    AP_Light(const AP_Light &other) = delete;
    AP_Light &operator=(const AP_Light&) = delete;

    // get singleton instance
    static AP_Light *instance(void) {
        return _instance;
    }

    // initialisation
    void init(void);

    /// update - allow updates of leds that cannot be updated during a timed interrupt
    void update(void);

    // handle a LED_CONTROL message
    static void handle_led_control(mavlink_message_t* msg);

    // handle a PLAY_TUNE message
    static void handle_play_tune(mavlink_message_t* msg);

private:

    static AP_Light *_instance;
};
