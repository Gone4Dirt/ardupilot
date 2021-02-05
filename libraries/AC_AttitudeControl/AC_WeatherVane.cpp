/*
 * Aircraft Weathervane options common to vtol plane and copters
 */

#include <AP_HAL/AP_HAL.h>
#include "AC_WeatherVane.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Terrain/AP_Terrain.h>

const AP_Param::GroupInfo AC_WeatherVane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description{Copter}: Enable weather vaning.  When active, and the appropriate _OPTIONS bit is set for auto, or guided, the aircraft will yaw into wind once the vehicle is above WVANE_MIN_ALT, is not moving, and there has been no pilot input for 3 seconds.
    // @Description{Plane}: Enable weather vaning.  When active, the aircraft will automatically yaw into wind when in a VTOL position controlled mode. Pilot yaw commands overide the weathervaning action.
    // @Values: 0:Disabled,1:Nose into wind,2:Nose or tail into wind (tailsitters)
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_WeatherVane, _direction, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: GAIN
    // @DisplayName: Weathervaning gain
    // @Description: This controls the tendency to yaw to face into the wind. A value of 0.1 is to start with and will give a slow turn into the wind. Use a value of 0.4 for more rapid response. The weathervaning works by turning into the direction of roll.
    // @Range: 0 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GAIN", 2, AC_WeatherVane, _gain, 0.5),

    // @Param: MIN_ANG
    // @DisplayName: Weathervaning min angle
    // @Description: The minimum angle error, in degrees, before active weathervaning will start.
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("MIN_ANG", 3, AC_WeatherVane, _min_dz_ang_deg, 1),

    // @Param: MIN_ALT
    // @DisplayName: Weathervaning min altitude
    // @Description: Above this altitude weathervaning is permitted.  If terrain is enabled, AGL is used.  Set zero to ignore altitude.
    // @Range: 0 50
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN_ALT", 4, AC_WeatherVane, _min_alt, 2),

    AP_GROUPEND
};


// Constructor
AC_WeatherVane::AC_WeatherVane(const AP_InertialNav& inav) :
    _inav(inav)
{
    reset();
    AP_Param::setup_object_defaults(this, var_info);
}

bool AC_WeatherVane::get_yaw_rate_cds(float& yaw_rate, const int16_t pilot_yaw, const int16_t roll_cdeg, const int16_t pitch_cdeg)
{
    const uint32_t now = AP_HAL::millis();

    // If it has been longer than 5 seconds since last call back then reset the controller
    if (now - last_callback > 5000) {
        reset();
    }
    last_callback = now;

    if (!should_weathervane(pilot_yaw, roll_cdeg, pitch_cdeg, now)) {
        return false;
    }

    if (!active_msg_sent) {
        gcs().send_text(MAV_SEVERITY_INFO, "Weathervane Active");
        active_msg_sent = true;
    }

    // Calculate new yaw rate
    float output = fabsf(roll_cdeg) * _gain;

    // For nose in only direction, we add nose up pitch contribution to maintain a 
    // consistant yaw rate when the vehicle has its tail to the wind.
    if (pitch_cdeg > 0 && get_direction() == Direction::NOSE_IN) {
        output += pitch_cdeg * _gain;
    }

    /*
      Determine yaw direction

      If we can yaw nose or tail into wind we want to yaw in the direction
      oppose to the roll angle when we have nose high attitudes. The nose
      low behaviour is as per NOSE_IN behaviour.

      At this point output only has magnitude, not direction.
    */
    if (pitch_cdeg > 0 && get_direction() == Direction::NOSE_OR_TAIL_IN) {
        if (roll_cdeg > 0) {
            output *= -1.0f;
        } // else roll will be negative and we want the output to be positive

    } else if (pitch_cdeg < 0 || get_direction() == Direction::NOSE_IN) {
        // Yaw in the direction of the lowest 'wing'
        if (roll_cdeg < 0) {
            output *= -1.0f;
        }
    }

    // Force the controller to relax.  This can be called when landing.
    if (should_relax) {
        output = 0;
        // Always reset should relax. Maintain relaxed condition by persistant calls to relax()
        should_relax = false;
    }

    // Slew output
    last_output = 0.98f * last_output + 0.02f * output;

    // Limit yaw rate if limits are set
    if (!is_zero(yaw_rate_max_deg_s)) {
        last_output = constrain_float(last_output, -yaw_rate_max_deg_s, yaw_rate_max_deg_s);
    }

    yaw_rate = last_output;

    return true;
}

// Called on an interupt to reset the weathervane controller
void AC_WeatherVane::reset(void)
{
    last_output = 0;
    active_msg_sent = false;
    should_relax = false;
    first_activate_ms = 0;
}

bool AC_WeatherVane::should_weathervane(int16_t pilot_yaw, int16_t roll_cdeg, int16_t pitch_cdeg, const uint32_t now)
{
    // Check enabled
    if ((Direction)((uint8_t)_direction) == Direction::OFF){
        reset();
        return false;
    }

    // Don't activate weather vaning if less than deadzone angle
    if (fabsf(roll_cdeg) < _min_dz_ang_deg*100.0f  &&  (pitch_cdeg > 0.0f || get_direction() != Direction::NOSE_IN)) {
        reset();
        return 0.0f;
    }

    // Don't fight pilot inputs
    if (pilot_yaw != 0) {
        last_pilot_input_ms = now;
        reset();
        return false;
    }

    // Only allow weather vaning if no input from pilot in last 3 seconds
    if (now - last_pilot_input_ms < 3000) {
        reset();
        return false;
    }

    // Check if we are above the minimum altitude to weather vane
    if (below_min_alt()) {
        reset();
        return false;
    }

    /*
      Use a 2 second buffer to ensure weathervaning occurs once the vehicle has
      clearly settled in an acceptable condition. This avoids weathervane
      activation as soon as a wp is accepted
    */
    if (first_activate_ms == 0) {
        first_activate_ms = now;
    }
    if (now - first_activate_ms < 2000){
        return false;
    }

    // If we got this far then we should allow weathervaning
    return true;
}

bool AC_WeatherVane::below_min_alt(void)
{
    // Check min altitude is set
    if (_min_alt.get() <= 0) {
        return false;
    }

#if AP_TERRAIN_AVAILABLE
    AP_Terrain* terrain = AP_Terrain::get_singleton();
    if (terrain != nullptr) {
        float terr_alt = 0.0f;
        if (terrain->enabled() && terrain->height_above_terrain(terr_alt, true) && terr_alt >= (float)_min_alt.get()) {
            return false;
        }
    }
#endif

    if (_inav.get_altitude() >= (float)_min_alt.get()) {
        return false;
    }

    return true;
}

AC_WeatherVane::Direction AC_WeatherVane::get_direction() const
{
    return (Direction)_direction.get();
}

// Get the AC_WeatherVane singleton
AC_WeatherVane *AC_WeatherVane::get_singleton()
{
    return _singleton;
}

AC_WeatherVane *AC_WeatherVane::_singleton = nullptr;
