/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  battery BMS includes a button which, when pressed, shows the state of charge percentage using LEDs
 */
#include "AP_Periph.h"

#if AP_PERIPH_BATTERY_BMS_ENABLED
#include "stdio.h"
#include "battery_bms.h"

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

BatteryBMS::BatteryBMS(void)
{
    // Initialize startup LED variables
    init_stage = 0;
    init_done = false;
    
    // Initialize BMS state machine
    bms_state = BmsState::IDLE;
    led_animation_step = 0;
    error_flash_count = 0;
    led_last_update_ms = 0;
    
    // Initialize button state variables
    button_last_state = true;  // Assuming button is pulled up (true = not pressed)
    button_press_start_ms = 0;
    button_press_handled = false;
    startup_complete = false;
    
    // Initialize LED display variables
    leds_displaying = false;
    led_display_start_ms = 0;

    // initialise LEDs
#if HAL_GPIO_LED_ON != 0
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED1, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED2, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED3, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED4, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED5, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED6, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED7, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED8, HAL_GPIO_OUTPUT);
    
    // Configure button as input with pullup
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_BTN1, HAL_GPIO_INPUT);
    hal.gpio->write(HAL_GPIO_PIN_BMS_BTN1, 1); // Enable pullup
#endif
}

void BatteryBMS::update(void)
{
#ifdef HAL_GPIO_PIN_BMS_BTN1
    handle_button_press();
#endif

    // Update LED state machine
    update_led_state();
}

#ifdef HAL_GPIO_PIN_BMS_BTN1
// Display battery percentage using 8 LEDs
// Each LED represents 12.5% (100% / 8 LEDs)
void BatteryBMS::display_percentage(uint8_t percentage)
{
    // Clamp percentage to 0-100
    if (percentage > 100) {
        percentage = 100;
    }
    
    // Calculate how many LEDs to light up
    // 0-12%: 1 LED, 13-25%: 2 LEDs, 26-37%: 3 LEDs, etc.
    uint8_t num_leds = (percentage + 12) / 13;  // Round up: (percentage / 12.5) rounded up
    if (num_leds > 8) {
        num_leds = 8;
    }
    
    // Turn on the appropriate number of LEDs
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED1, num_leds >= 1 ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED2, num_leds >= 2 ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED3, num_leds >= 3 ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED4, num_leds >= 4 ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED5, num_leds >= 5 ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED6, num_leds >= 6 ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED7, num_leds >= 7 ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED8, num_leds >= 8 ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    
    // Start LED display timer
    leds_displaying = true;
    led_display_start_ms = AP_HAL::millis();
}

// Get battery percentage (0-100)
// Returns true if percentage was obtained, false otherwise
bool BatteryBMS::get_percentage(uint8_t &percentage)
{
    percentage = 0;
    
    if (periph.battery_lib.num_instances() == 0) {
        return false;
    }
    
    // Try to get capacity remaining percentage first
    if (periph.battery_lib.capacity_remaining_pct(percentage, 0)) {
        return true;
    }
    
    // Fallback: Calculate percentage from average cell voltage
    // Li-ion/LiPo typical range: 3.0V (0%) to 4.2V (100%)
    if (periph.battery_lib.has_cell_voltages(0)) {
        const AP_BattMonitor::cells &cell_voltages = periph.battery_lib.get_cell_voltages(0);
        uint32_t total_voltage_mv = 0;
        uint8_t cell_count = 0;
        
        for (uint8_t i = 0; i < AP_BATT_MONITOR_CELLS_MAX; i++) {
            if (cell_voltages.cells[i] == UINT16_MAX) {
                break;
            }
            total_voltage_mv += cell_voltages.cells[i];
            cell_count++;
        }
        
        if (cell_count > 0) {
            uint16_t avg_cell_voltage_mv = total_voltage_mv / cell_count;
            // Map 3000mV-4200mV to 0-100%
            if (avg_cell_voltage_mv <= 3000) {
                percentage = 0;
            } else if (avg_cell_voltage_mv >= 4200) {
                percentage = 100;
            } else {
                percentage = ((avg_cell_voltage_mv - 3000) * 100) / 1200;
            }
            return true;
        }
    }
    
    return false;
}

void BatteryBMS::handle_button_press(void)
{
    uint32_t now_ms = AP_HAL::millis();
    
    // Ignore button presses during startup to avoid false detections from GPIO initialization
    if (!startup_complete) {
        if (now_ms < STARTUP_DELAY_MS) {
            return;
        }
        startup_complete = true;
    }
    
    // Read current button state (assuming active low - pressed = 0)
    bool button_raw_state = hal.gpio->read(HAL_GPIO_PIN_BMS_BTN1);
    bool button_current_state = (button_raw_state == 0); // pressed = true
    
    // Button just pressed (transition from released to pressed)
    if (button_current_state && !button_last_state) {
        button_press_start_ms = now_ms;
        button_press_handled = false;
        printf("BMS: Button pressed!\n");
    }
    
    // Button is currently pressed - check for long press
    if (button_current_state && !button_press_handled) {
        // Calculate press duration only when button is pressed
        uint32_t press_duration = now_ms - button_press_start_ms;
        
        // Long press detected (1+ seconds)
        if (press_duration >= LONG_PRESS_THRESHOLD_MS) {
            button_press_handled = true;
            
            // Handle state transitions based on BMS state
            if (periph.battery_lib.num_instances() > 0) {
                auto power_state = periph.battery_lib.get_power_state(0);
                
                switch(bms_state) {
                    case BmsState::IDLE:
                        // Only allow power on if battery is in IDLE power state
                        if (power_state == AP_BattMonitor::PowerState::IDLE) {
                            printf("BMS: Long press confirmed - starting discharge\n");
                            bms_state = BmsState::POWERING_ON_CONFIRMED;
                            led_animation_step = 0;
                        } else {
                            printf("BMS: Long press ignored (battery not in IDLE power state)\n");
                        }
                        break;
                        
                    case BmsState::POWERED_ON:
                        // Allow power off from POWERED_ON state regardless of power state
                        printf("BMS: Long press confirmed - stopping discharge\n");
                        bms_state = BmsState::POWERING_OFF_CONFIRMED;
                        led_animation_step = 0;
                        break;
                        
                    case BmsState::POWERING_ON:
                    case BmsState::POWERING_ON_CONFIRMED:
                    case BmsState::POWERING_OFF:
                    case BmsState::POWERING_OFF_CONFIRMED:
                        // Already in transition, ignore
                        break;
                        
                    default:
                        break;
                }
            }
        }
    }
    
    // Button just released (transition from pressed to released)
    if (!button_current_state && button_last_state) {
        // Check if button released during confirmed state - complete the action
        if (bms_state == BmsState::POWERING_ON_CONFIRMED) {
            printf("BMS: Button released - completing power on\n");
            // Enable discharge immediately
            if (periph.battery_lib.num_instances() > 0) {
                periph.battery_lib.set_discharge(0, true);
                printf("BMS: Discharge ENABLED\n");
            }
            bms_state = BmsState::POWERING_ON;
            led_animation_step = 0;
            button_press_handled = false;
            button_last_state = button_current_state;
            return;
        }
        if (bms_state == BmsState::POWERING_OFF_CONFIRMED) {
            printf("BMS: Button released - completing power off\n");
            // Disable discharge immediately
            if (periph.battery_lib.num_instances() > 0) {
                periph.battery_lib.set_discharge(0, false);
                printf("BMS: Discharge DISABLED\n");
            }
            bms_state = BmsState::POWERING_OFF;
            led_animation_step = 0;
            button_press_handled = false;
            button_last_state = button_current_state;
            return;
        }
        
        // Calculate press duration for short press detection
        uint32_t press_duration = now_ms - button_press_start_ms;
        
        // Only process if press duration is reasonable (at least 10ms to avoid noise)
        if (press_duration >= 10) {
            // Short press detected (less than 1 second and not already handled as long press)
            // Only process short press when in IDLE state
            if (press_duration < LONG_PRESS_THRESHOLD_MS && !button_press_handled) {
                if (periph.battery_lib.num_instances() > 0) {
                    auto power_state = periph.battery_lib.get_power_state(0);
                    if (power_state == AP_BattMonitor::PowerState::DISCHARGING ||
                        power_state == AP_BattMonitor::PowerState::CHARGING) {
                        printf("BMS: Short press ignored (not in IDLE state)\n");
                    } else {
                        printf("BMS: SHORT PRESS DETECTED! (%d ms)\n", (int)press_duration);
                    
                        // Display battery percentage on LEDs and print cell voltages
                        uint8_t num_instances = periph.battery_lib.num_instances();
                
                        if (num_instances == 0) {
                            printf("BMS: ERROR - No battery monitor instances found!\n");
                        } else {
                            uint8_t percentage = 0;
                            if (get_percentage(percentage)) {
                                printf("Battery capacity remaining: %d%%\n", percentage);
                                display_percentage(percentage);
                                const AP_BattMonitor::cells &cell_voltages = periph.battery_lib.get_cell_voltages(0);
                                printf("Cell voltages: ");
                                for (uint8_t i = 0; i < AP_BATT_MONITOR_CELLS_MAX; i++) {
                                    if (cell_voltages.cells[i] == UINT16_MAX) {
                                        break;  // End of valid cells
                                    }
                                    printf("Cell%d=%dmV ", i+1, cell_voltages.cells[i]);
                                }
                                printf("Total=%.3fV\n", periph.battery_lib.voltage(0));
                            }
                        }
                    }
                }
            }
        }
        
        // If it was a long press
        if (button_press_handled) {
            printf("BMS: Long press released after %d ms\n", (int)press_duration);
        }
        
        button_press_handled = false;
    }

    // Update last state for next iteration
    button_last_state = button_current_state;
}

// Set LED pattern based on 8-bit bitmask
void BatteryBMS::set_led_pattern(uint8_t pattern)
{
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED1, (pattern & 0x01) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED2, (pattern & 0x02) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED3, (pattern & 0x04) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED4, (pattern & 0x08) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED5, (pattern & 0x10) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED6, (pattern & 0x20) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED7, (pattern & 0x40) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_PIN_BMS_LED8, (pattern & 0x80) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
}

// LED state machine - manages LED display based on battery state
void BatteryBMS::update_led_state(void)
{
    uint32_t now_ms = AP_HAL::millis();
    
    // If displaying SOC, don't update state machine
    if (leds_displaying) {
        if (now_ms - led_display_start_ms >= LED_DISPLAY_DURATION_MS) {
            leds_displaying = false;
            // Turn off all LEDs after displaying percentage
            set_led_pattern(0x00);
            // Will update to current battery state below
        } else {
            return; // Keep displaying SOC
        }
    }
    
    // Handle POWERING_ON_CONFIRMED state - turn on LEDs one by one while holding button
    if (bms_state == BmsState::POWERING_ON_CONFIRMED) {
        if (now_ms - led_last_update_ms < LED_UPDATE_INTERVAL_MS) {
            return;
        }
        led_last_update_ms = now_ms;
        
        if (led_animation_step <= 8) {
            led_animation_step++;
            if (led_animation_step <= 8) {
                // Turn on LEDs sequentially
                set_led_pattern((1 << led_animation_step) - 1);
            } else {
                // Keep all LEDs on after animation completes
                set_led_pattern(0xFF);
            }
        }
        return;
    }
    
    // Handle POWERING_ON state - transition immediately to POWERED_ON
    if (bms_state == BmsState::POWERING_ON) {
        bms_state = BmsState::POWERED_ON;
        led_animation_step = 0;
        init_done = true;
        return;
    }
    
    // Handle POWERED_ON state - keep all LEDs on
    if (bms_state == BmsState::POWERED_ON) {
        set_led_pattern(0xFF); // All LEDs on
        return;
    }
    
    // Handle ERROR_FLASH state - flash all LEDs 3 times
    if (bms_state == BmsState::ERROR_FLASH) {
        if (now_ms - led_last_update_ms < LED_UPDATE_INTERVAL_MS) {
            return;
        }
        led_last_update_ms = now_ms;
        
        // Flash pattern: on/off/on/off/on/off (6 steps = 3 flashes)
        if (led_animation_step < 6) {
            if (led_animation_step % 2 == 0) {
                set_led_pattern(0xFF); // All LEDs on
            } else {
                set_led_pattern(0x00); // All LEDs off
            }
            led_animation_step++;
        } else {
            // Flashing complete - return to IDLE
            set_led_pattern(0x00);
            bms_state = BmsState::IDLE;
            led_animation_step = 0;
        }
        return;
    }
    
    // Handle POWERING_OFF_CONFIRMED state - turn off LEDs one by one while holding button
    if (bms_state == BmsState::POWERING_OFF_CONFIRMED) {
        if (now_ms - led_last_update_ms < LED_UPDATE_INTERVAL_MS) {
            return;
        }
        led_last_update_ms = now_ms;
        
        if (led_animation_step == 0) {
            led_animation_step = 8; // Start from 8 LEDs on
            set_led_pattern(0xFF); // All LEDs on
        } else if (led_animation_step > 0 && led_animation_step <= 8) {
            led_animation_step--;
            if (led_animation_step > 0) {
                set_led_pattern((1 << led_animation_step) - 1);
            } else {
                // Animation complete - keep all LEDs off
                set_led_pattern(0x00);
                led_animation_step = 255; // Mark as complete to prevent restart
            }
        }
        return;
    }
    
    // Handle POWERING_OFF state - transition immediately to IDLE
    if (bms_state == BmsState::POWERING_OFF) {
        set_led_pattern(0x00); // All off
        bms_state = BmsState::IDLE;
        return;
    }
    
    // Get current power state from battery library
    if (periph.battery_lib.num_instances() > 0) {
        auto power_state = periph.battery_lib.get_power_state(0);
        
        // Run animations for CHARGING and DISCHARGING states
        if (power_state == AP_BattMonitor::PowerState::CHARGING || power_state == AP_BattMonitor::PowerState::DISCHARGING) {
            if (now_ms - led_last_update_ms < LED_UPDATE_INTERVAL_MS) {
                return;
            }
            led_last_update_ms = now_ms;
            
            led_animation_step = (led_animation_step + 1) % 8;
            
            // CHARGING: chase forward (bit 0 -> 7)
            // DISCHARGING: chase backward (bit 7 -> 0)
            uint8_t pattern = (power_state == AP_BattMonitor::PowerState::CHARGING) ?
                              (1 << led_animation_step) :
                              (0x80 >> led_animation_step);
            
            set_led_pattern(pattern);
        }
    }
}
#endif

#endif  // AP_PERIPH_BATTERY_BMS_ENABLED
