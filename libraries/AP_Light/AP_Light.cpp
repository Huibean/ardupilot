#include "AP_Light.h"
#include <stdio.h>

//extern const AP_HAL::HAL& hal;

AP_Light *AP_Light::_instance;

// Default constructor
AP_Light::AP_Light()
{
    if (_instance != nullptr) {
        AP_HAL::panic("AP_Notify must be singleton");
    }
    _instance = this;
} 

// initialisation
void AP_Light::init(void)
{
  printf("AP_Light::init \n");
}

// main update function, called at 50Hz
void AP_Light::update(void)
{
  //printf("AP_Light::update \n");
}

// handle a LED_CONTROL message
void AP_Light::handle_led_control(mavlink_message_t *msg)
{
}