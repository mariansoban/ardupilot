/*
  Buzzer driver
*/
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
#include "Buzzer.h"

#include <AP_HAL/AP_HAL.h>

#include "AP_Notify.h"

#ifndef HAL_BUZZER_ON
  #if !defined(HAL_BUZZER_PIN)
    #define HAL_BUZZER_ON (pNotify->get_buzz_level())
    #define HAL_BUZZER_OFF (!pNotify->get_buzz_level())
  #else
    #define HAL_BUZZER_ON 1
    #define HAL_BUZZER_OFF 0 
  #endif
#endif



extern const AP_HAL::HAL& hal;


bool Buzzer::init()
{
    if (pNotify->buzzer_enabled() == false) {
        return false;
    }
#if defined(HAL_BUZZER_PIN)
    _pin = HAL_BUZZER_PIN;
#else
    _pin = pNotify->get_buzz_pin();
#endif
    if(!_pin) return false;

    // setup the pin and ensure it's off
    hal.gpio->pinMode(_pin, HAL_GPIO_OUTPUT);
    on(false);

    // set initial boot states. This prevents us issuing a arming
    // warning in plane and rover on every boot
    _flags.armed = AP_Notify::flags.armed;
    _flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
    
    on(false); // XXX [ms] PHL buzzer FIX - ensure twice more, that buzzer pin is in off state
    hal.gpio->write(_pin, HAL_BUZZER_OFF);
    // Note: Don't forget to set NTF_BUZZ_PIN to 55 for PHL for buzzer on AUX6
    return true;
}

// update - updates led according to timed_updated.  Should be called at 50Hz
void Buzzer::update()
{
    update_pattern_to_play();
    update_playing_pattern();
}

void Buzzer::update_pattern_to_play()
{
    // check for arming failed event
    if (AP_Notify::events.arming_failed) {
        // arming failed buzz
        play_pattern(SINGLE_BUZZ);
        return;
    }

    if (AP_HAL::millis() - _pattern_start_time < _pattern_start_interval_time_ms) {
        // do not interrupt playing patterns / enforce minumum separation
        return;
    }

    // check if armed status has changed
    if (_flags.armed != AP_Notify::flags.armed) {
        _flags.armed = AP_Notify::flags.armed;
        if (_flags.armed) {
            // double buzz when armed
            play_pattern(ARMING_BUZZ);
        }else{
            // single buzz when disarmed
            play_pattern(SINGLE_BUZZ);
        }
        return;
    }

    // check ekf bad
    if (_flags.ekf_bad != AP_Notify::flags.ekf_bad) {
        _flags.ekf_bad = AP_Notify::flags.ekf_bad;
        if (_flags.ekf_bad) {
            // ekf bad warning buzz
            play_pattern(EKF_BAD);
        }
        return;
    }

    // if vehicle lost was enabled, starting beep
    if (AP_Notify::flags.vehicle_lost) {
        play_pattern(DOUBLE_BUZZ);
        return;
    }

    // if battery failsafe constantly single buzz
    if (AP_Notify::flags.failsafe_battery) {
        play_pattern(SINGLE_BUZZ);
        return;
    }


    // XXX [ms] PHL buzzer FIX
    // gps
    switch (AP_Notify::flags.gps_status) {
        case 0:
            // no GPS attached
            break;

        case 1:
            // GPS attached but no lock, blink at 4Hz
            break;

        case 2:
            // GPS attached but 2D lock, blink more slowly (around 2Hz)
            break;

        case 3:
            // GPS 3D lock
            if (_last_gps_status != AP_Notify::flags.gps_status) {
                play_pattern(DOUBLE_BUZZ);
            }
            break;
        default:
            // GPS DGPS (4) or RTK (5) lock
            if (_last_gps_status != AP_Notify::flags.gps_status) {
                play_pattern(TRIPLE_BUZZ);
            }
            break;
    }
    _last_gps_status = AP_Notify::flags.gps_status;
}


void Buzzer::update_playing_pattern()
{
    if (_pattern == 0UL) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    const uint32_t delta = now - _pattern_start_time;
    if (delta >= 3200) {
        // finished playing pattern
        on(false);
        _pattern = 0UL;
        return;
    }
    const uint32_t bit = delta / 100UL; // each bit is 100ms
    on(_pattern & (1U<<(31-bit)));
}

// on - turns the buzzer on or off
void Buzzer::on(bool turn_on)
{
    // return immediately if nothing to do
    if (_flags.on == turn_on) {
        return;
    }

    // update state
    _flags.on = turn_on;

    // pull pin high or low
    hal.gpio->write(_pin, _flags.on? HAL_BUZZER_ON : HAL_BUZZER_OFF);
}

/// play_pattern - plays the defined buzzer pattern
void Buzzer::play_pattern(const uint32_t pattern)
{
    _pattern = pattern;
    _pattern_start_time = AP_HAL::millis();
}

