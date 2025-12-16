#pragma once

#if AP_PERIPH_BATTERY_BMS_ENABLED

class BatteryBMS {
public:
    friend class AP_Periph_FW;
    BatteryBMS(void);

    void update(void);

private:

    // handle button press
    void handle_button_press(void);
    
    // display battery percentage using 8 LEDs
    void display_percentage(uint8_t percentage);
    
    // get battery percentage (0-100)
    bool get_percentage(uint8_t &percentage);
    
    // BMS animation state
    enum class BmsState : uint8_t {
        IDLE = 0,
        POWERING_ON,
        POWERING_ON_CONFIRMED,
        POWERED_ON,
        POWERING_OFF_CONFIRMED,
        POWERING_OFF,
        ERROR_FLASH
    };
    
    void update_led_state(void);
    void set_led_pattern(uint8_t pattern);

    // startup LED variables
    uint8_t init_stage; // current stage of the startup LED sequence
    bool init_done;     // true once the startup LED sequence has completed
    
    // BMS state machine variables  
    BmsState bms_state;
    uint8_t led_animation_step;
    uint8_t error_flash_count;
    uint32_t led_last_update_ms;
    static const uint32_t LED_UPDATE_INTERVAL_MS = 150;

    // Button handling variables
    bool button_last_state;
    uint32_t button_press_start_ms;
    bool button_press_handled;
    bool startup_complete;
    static const uint32_t LONG_PRESS_THRESHOLD_MS = 1000; // 2 seconds for long press
    static const uint32_t STARTUP_DELAY_MS = 2000; // Ignore button presses for first 2 seconds
    
    // LED display variables
    bool leds_displaying;
    uint32_t led_display_start_ms;
    static const uint32_t LED_DISPLAY_DURATION_MS = 1000; // Display LEDs for 1 second
};

#endif // AP_PERIPH_BATTERY_BMS_ENABLED

