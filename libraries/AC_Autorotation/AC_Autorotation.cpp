#include "AC_Autorotation.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

// Head Speed (HS) controller specific default definitions
#define HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ          100.0f    // low-pass filter collective output
#define HS_CONTROLLER_HEADSPEED_P                     0.7f     // Default P gain for head speed controller (unit: -)
#define HEAD_SPEED_TARGET_RATIO                       1.0f     // Normalised target main rotor head speed

// Speed Height controller specific default definitions for autorotation use
#define AP_FW_VEL_P                                   0.9f    // Default forward speed controller P gain
#define TCH_P                                         0.1f    // Default touchdown phase collective controller P gain

// flare controller default definitions
#define AP_ALPHA_TPP                                  20.0f   // (deg) Maximum angle of the Tip Path Plane
#define MIN_TIME_ON_GROUND                            3000    // (ms) Time on ground required before collective is slewed to zero thrust
#define FLARE_TEST_PRINT_INTERVAL                     5000    // (ms) Time between sending updates to the flare estimate calculation to the GCS

const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_Autorotation, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_P
    // @DisplayName: P gain for head speed controller
    // @Description: Increase value to increase sensitivity of head speed controller during autonomous autorotation.
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_p_hs, "HS_", 2, AC_Autorotation, AC_P),

    // @Param: HS_SET_PT
    // @DisplayName: Target Head Speed
    // @Description: The target head speed in RPM during autorotation.  Start by setting to desired hover speed and tune from there as necessary.
    // @Units: RPM
    // @Range: 1000 2800
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("HS_SET_PT", 3, AC_Autorotation, _param_head_speed_set_point, 1500),

    // @Param: TARG_SP
    // @DisplayName: Target Glide Ground Speed
    // @Description: Target ground speed in cm/s for the autorotation controller to try and achieve/ maintain during the glide phase.
    // @Units: cm/s
    // @Range: 800 2000
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("TARG_SP", 4, AC_Autorotation, _param_target_speed, 1100),

    // @Param: COL_FILT_E
    // @DisplayName: Entry Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the entry phase.  Acts as a following trim.  Must be higher than AROT_COL_FILT_G.
    // @Units: Hz
    // @Range: 0.2 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_E", 5, AC_Autorotation, _param_col_entry_cutoff_freq, 0.7),

    // @Param: COL_FILT_G
    // @DisplayName: Glide Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the glide phase.  Acts as a following trim.  Must be lower than AROT_COL_FILT_E.
    // @Units: Hz
    // @Range: 0.03 0.15
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_G", 6, AC_Autorotation, _param_col_glide_cutoff_freq, 0.1),

    // @Param: AS_ACC_MAX
    // @DisplayName: Forward Acceleration Limit
    // @Description: Maximum forward acceleration to apply in speed controller.
    // @Units: cm/s/s
    // @Range: 30 60
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("AS_ACC_MAX", 7, AC_Autorotation, _param_accel_max, 60),

    // @Param: BAIL_TIME
    // @DisplayName: Bail Out Timer
    // @Description: Time in seconds from bail out initiated to the exit of autorotation flight mode.
    // @Units: s
    // @Range: 0.5 4
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BAIL_TIME", 8, AC_Autorotation, _param_bail_time, 2.0),

    // @Param: HS_SENSOR
    // @DisplayName: Main Rotor RPM Sensor
    // @Description: Allocate the RPM sensor instance to use for measuring head speed.  RPM1 = 0.  RPM2 = 1.
    // @Units: s
    // @Range: 0.5 3
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("HS_SENSOR", 9, AC_Autorotation, _param_rpm_instance, 0),

    // @Param: FW_V_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Determines the proportion of the target acceleration based on the velocity error.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced
    AP_SUBGROUPINFO(_p_fw_vel, "FW_V_", 10, AC_Autorotation, AC_P),

    // @Param: FW_V_FF
    // @DisplayName: Velocity (horizontal) feed forward
    // @Description: Velocity (horizontal) input filter.  Corrects the target acceleration proportionally to the desired velocity.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("FW_V_FF", 11, AC_Autorotation, _param_fwd_k_ff, 0.15),

    // @Param: TCH_P
    // @DisplayName: P gain for vertical touchdown controller
    // @Description: proportional term based on sink rate error
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_p_coll_tch, "TCH_", 12, AC_Autorotation, AC_P),

    // @Param: COL_FILT_C
    // @DisplayName: Touchdown Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the touchdown phase.  Acts as a following trim.
    // @Units: Hz
    // @Range: 0.2 0.8
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_C", 13, AC_Autorotation, _param_col_touchdown_cutoff_freq, 0.5),

    // @Param: ROT_SOL
    // @DisplayName: rotor solidity
    // @Description: helicopter specific main rotor solidity
    // @Range: 0.001 0.01
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ROT_SOL", 14, AC_Autorotation, _param_solidity, 0.05),

    // @Param: ROT_DIAM
    // @DisplayName: rotor diameter
    // @Description: helicopter specific main rotor diameter
    // @Units: m
    // @Range: 0.001 0.01
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ROT_DIAM", 15, AC_Autorotation, _param_diameter, 1.25),

    // @Param: T_TCH
    // @DisplayName: time touchdown
    // @Description: time touchdown
    // @Units: s
    // @Range: 0.001 0.01
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("T_TCH", 16, AC_Autorotation, _param_touchdown_time, 1.0),

    // @Param: OPTIONS
    // @DisplayName: Autorotation options
    // @Description: Bitmask for autorotation options.
    // @Bitmask: 0: Use stabilize-like controls (roll angle, yaw rate), 1: Print inital flare calc, 2: Test rangefinder timeout
    AP_GROUPINFO("OPTIONS", 17, AC_Autorotation, _options, 0),

    // @Param: CL_ALPHA
    // @DisplayName: Rotor averaged lift curve slope
    // @Description: The mean lift curve slope of the rotor. Defines the change in lift coefficient with respect to a reference blade pitch angle. This param cannot have a value higher than the theoretical maximum of 2 pi.
    // @Range: 1.6 6.283
    AP_GROUPINFO("CL_ALPHA", 18, AC_Autorotation, _c_l_alpha, M_2PI),

    AP_GROUPEND
};


// Constructor
AC_Autorotation::AC_Autorotation(AP_InertialNav& inav, AP_AHRS& ahrs, AP_MotorsHeli*& motors):
    _inav(inav),
    _ahrs(ahrs),
    _motors_heli(motors),
    _p_hs(HS_CONTROLLER_HEADSPEED_P),
    _p_fw_vel(AP_FW_VEL_P),
    _p_coll_tch(TCH_P)
{
    AP_Param::setup_object_defaults(this, var_info);
}


void AC_Autorotation::init() {
    // Reset z acceleration average variables
    _avg_acc_z = 0.0f;
    _acc_z_sum = 0.0f;
    _index = 0;
    memset(_curr_acc_z, 0, sizeof(_curr_acc_z));

    initial_flare_estimate();

    // Initialisation of head speed controller
    // Set initial collective position to be the zero thrust collective and minimize loss in head speed
    _collective_out = _motors_heli->get_coll_mid();

    // Reset feed forward filter
    col_trim_lpf.reset(_collective_out);

    // Protect against divide by zero
    _param_head_speed_set_point.set(MAX(_param_head_speed_set_point, 500.0));

    // Initialise forward speed controller
    _accel_target = 0.0;

    // Ensure parameter acceleration doesn't exceed hard-coded limit
    _accel_max = MIN(_param_accel_max, 500.0);

    // Reset cmd vel and last accel to sensible values
    _cmd_vel = calc_speed_forward(); //(cm/s)
    _accel_out_last = _cmd_vel * _param_fwd_k_ff;

    // reset on ground timer
    _time_on_ground = 0;

    // Retrieve rpm and start rpm sensor health checks
    _initial_norm_rpm = get_norm_rpm();

    // The decay rate to reduce the head speed from the current to the target
    _hs_decay = (_initial_norm_rpm - HEAD_SPEED_TARGET_RATIO) / (float(entry_time_ms)*1e-3);
}

// Functions and config that are only to be done once at the beginning of the entry
void AC_Autorotation::init_entry(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");

    // Set desired forward speed target
    _vel_target = _param_target_speed.get();

    // Target head speed is set to rpm at initiation to prevent steps in controller
    _target_head_speed = _initial_norm_rpm;

    // Set collective following trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_entry_cutoff_freq.get());

    // set collective 
    _motors_heli->set_throttle_filter_cutoff(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);
}

// The entry controller just a special case of the glide controller with head speed target slewing
void AC_Autorotation::run_entry(float& pitch_target)
{
    // Slowly change the target head speed until the target head speed matches the parameter defined value
    float norm_rpm = get_norm_rpm();
    if (norm_rpm > HEAD_SPEED_TARGET_RATIO*1.005f  ||  norm_rpm < HEAD_SPEED_TARGET_RATIO*0.995f) {
        // Outside of 5% of target head speed so we slew target towards the set point
        _target_head_speed -= _hs_decay * _dt;
    } else {
        _target_head_speed = HEAD_SPEED_TARGET_RATIO;
    }

    run_glide(pitch_target);
}

// Functions and config that are only to be done once at the beginning of the hover auto entry
void AC_Autorotation::init_hover_entry()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Hover Entry Phase");

    // Set collective trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_entry_cutoff_freq.get());

    // Reset the following trim filter to the current collective that is being output for smooth init
    col_trim_lpf.reset(_motors_heli->get_throttle());

    // set pitch target to current state for smooth transfer
    _pitch_target = _ahrs.get_pitch();
}

// Controller phase where the aircraft is too low and too slow to perform a full autorotation
// instead, we will try to minimize rotor drag until we can jump to the touch down phase
void AC_Autorotation::run_hover_entry(float& pitch_target)
{
    // Slewing collective with following trim filter, from current to zero thrust to minimize rotor speed loss
    _ff_term_hs = col_trim_lpf.apply(_motors_heli->get_coll_mid(), _dt);

    // Calculate collective position to be set
    _collective_out = constrain_value(_ff_term_hs, 0.0f, 1.0f);

    // Send collective setting to motors
    _motors_heli->set_throttle(_collective_out);

    // Smoothly move vehicle pitch angle to level over half a second (*2.0 = /0.5)
    _pitch_target *= 1 - (_dt * 2.0);
    pitch_target = _pitch_target;
}

// Functions and config that are only to be done once at the beginning of the glide
void AC_Autorotation::init_glide(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Glide Phase");

    // Set collective following trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_glide_cutoff_freq.get());

    // Ensure target headspeed is set to setpoint, in case it didn't reach the target during entry
    _target_head_speed = HEAD_SPEED_TARGET_RATIO;

    // Ensure desired forward speed target is set to param value
    _vel_target = _param_target_speed.get();
}

// Maintain head speed and forward speed as we glide to the ground
void AC_Autorotation::run_glide(float& pitch_target)
{
    update_headspeed_controller();

    update_forward_speed_controller(pitch_target);

    // Keep flare altitude estimate up to date so state machine can decide when to flare
    update_flare_alt();
}

// Functions and config that are only to be done once at the beginning of the flare
void AC_Autorotation::init_flare(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Flare_Phase");

    // Ensure target head speed, we may have skipped the glide phase if we did not have time to complete the
    // entry phase before hitting the flare height
    _target_head_speed = HEAD_SPEED_TARGET_RATIO;

    _flare_entry_speed = calc_speed_forward();
}

void AC_Autorotation::run_flare(float& pitch_target)
{
    // Update head speed/ collective controller
    update_headspeed_controller();

    // During the flare we want to linearly slow the aircraft to a stop as we
    // reach the _cushion_alt for the start of the touch down phase
    _vel_target = linear_interpolate(0.0f, _flare_entry_speed, _hagl, _cushion_alt, _flare_alt_calc);

    // Run forward speed controller
    update_forward_speed_controller(pitch_target);

    // TODO: Play with this bit to figure out what it adds/does
    // Estimate flare effectiveness
    if (_speed_forward <= (0.6 * _flare_entry_speed) && (fabsf(_avg_acc_z+GRAVITY_MSS) <= 0.5f)) {
        if (!_flare_complete) {
            gcs().send_text(MAV_SEVERITY_INFO, "Flare_complete");
            _flare_complete = true;
        }
    }

    if (!_flare_complete) {
        _pitch_target = atanf(-_accel_out / (GRAVITY_MSS * 100.0f)) * (18000.0f/M_PI);
        _pitch_target = constrain_float(_pitch_target, 0.0f, AP_ALPHA_TPP * 100.0f);
    } else {
        _pitch_target *= 0.9995f;
    }
}

// Functions and config that are only to be done once at the beginning of the touchdown
void AC_Autorotation::init_touchdown(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Touchdown_Phase");

    // Set following trim low pass cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_touchdown_cutoff_freq.get());

    // store the descent speed and height at the start of the touch down
    _touchdown_init_sink_rate = _inav.get_velocity_z_up_cms();
    _touchdown_init_alt = _hagl;
}

// Ensure vehicle is level and use energy stored in head to gently touch down on the ground
void AC_Autorotation::run_touchdown(float& pitch_target)
{
    float _current_sink_rate = _inav.get_velocity_z_up_cms();
    if (_hagl >= 0.0) {
        _desired_sink_rate = linear_interpolate(0.0, _touchdown_init_sink_rate, _hagl, 0.0, _touchdown_init_alt);
    } else {
        _desired_sink_rate = 0.0;
    }

    // Update forward speed for logging
    _speed_forward = calc_speed_forward(); // (cm/s)

    // Check to see if we think the aircraft is on the ground
    if(_current_sink_rate < 10.0 && _hagl <= 0.0 && _time_on_ground == 0){
        _time_on_ground = AP_HAL::millis();
    }

    // Use a timer to get confidence that the aircraft is on the ground.
    // Note: The landing detector uses the zero thust collective as an indicator for being on the ground. The touch down controller will have
    // driven the collective high to cushion the landing so the landing detector will not trip until we drive the collective back to zero thrust and below.
    if ((_time_on_ground > 0) && ((AP_HAL::millis() - _time_on_ground) > MIN_TIME_ON_GROUND)) {
       // On ground, smoothly lower collective to land col min, to make sure we trip the landing detector by going past zero thrust collective
       _collective_out = _collective_out*0.9 + _motors_heli->get_coll_land_min() * 0.1;
       _pitch_target = 0.0;

    } else {

        // Smoothly move vehicle pitch angle to level over the expected touch down time. Constrain min touch down time to be half a second.
        float touch_down_time = MAX(_param_touchdown_time.get(), 0.5);
        _pitch_target *= 1 - (_dt / touch_down_time);

        // Update following trim component
        _ff_term_hs = col_trim_lpf.apply(_collective_out, _dt);

        // Update collective output
        // Output of p controller is scaled by 1e-3 as collective to sink rate is extremely sensitive, this makes the gain a more friendly number
        _collective_out =  (_p_coll_tch.get_p(_desired_sink_rate - _current_sink_rate)*1e-3) + _ff_term_hs;
        _collective_out = constrain_value(_collective_out, 0.0f, 1.0f);
    }

    pitch_target = _pitch_target;

    _motors_heli->set_throttle(_collective_out);
}


// Rotor Speed controller for entry, glide and flare phases of autorotation
void AC_Autorotation::update_headspeed_controller(void)
{
    // Get current rpm and update healthy signal counters
    float head_speed_norm = get_norm_rpm();

    if (head_speed_norm > 0.0) {
        // Calculate the head speed error.
        _head_speed_error = head_speed_norm - _target_head_speed;

        _p_term_hs = _p_hs.get_p(_head_speed_error);

        // Adjusting collective trim using feed forward (not yet been updated, so this value is the previous time steps collective position)
        _ff_term_hs = col_trim_lpf.apply(_collective_out, _dt);

        // Calculate collective position to be set
        _collective_out = constrain_value((_p_term_hs + _ff_term_hs), 0.0f, 1.0f) ;

    } else {
         // RPM sensor is bad, set collective to angle of -2 deg and hope for the best
         _collective_out = _motors_heli->calc_coll_from_ang(-2.0);
    }

    // Send collective to setting to motors output library
    _motors_heli->set_throttle(_collective_out);
}


// Function that gets rpm and does rpm signal checking to ensure signal is reliable
// before using it in the controller
float AC_Autorotation::get_norm_rpm(void)
{
    float current_rpm = 0.0;

#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        // Check requested rpm instance to ensure either 0 or 1. Always defaults to 0.
        if ((_param_rpm_instance > 1) || (_param_rpm_instance < 0)) {
            _param_rpm_instance.set(0);
        }

        // Get RPM value
        uint8_t instance = _param_rpm_instance;

        // Check RPM sensor is returning a healthy status
        if (!rpm->get_rpm(instance, current_rpm)) {
            current_rpm = 0.0;
        }
    }
#endif

    // Protect against div by zeros later in the code
    float head_speed_set_point = MAX(1.0, _param_head_speed_set_point.get());

    // Normalize the RPM by the setpoint
    return current_rpm/head_speed_set_point;
}

// Helper to continuously print the flare estimate calculations to the GCS.
// This helps users to tune some of their params without having to actually fly an autorotation.
void AC_Autorotation::run_flare_prelim_calc(void)
{
    if (!_param_enable.get() || !(_options.get() & int32_t(OPTION::PRINT_FLARE_ESTIMATES))) {
        return;
    }

    if (AP_HAL::millis() - _last_flare_test_ms > FLARE_TEST_PRINT_INTERVAL) {
        // Run the initial flare estimate to get the GCS prints
        initial_flare_estimate();
        _last_flare_test_ms = AP_HAL::millis();
    }

}


#define ASSUMED_CD0 0.011
void AC_Autorotation::initial_flare_estimate(void)
{
    // Get blade pitch angle, accounting for non-zero zero thrust blade pitch for the asymmetric blade case
    float blade_pitch_hover_rad = radians(_motors_heli->get_hover_coll_ang() - _motors_heli->get_coll_zero_thrust_pitch());

    // Ensure safe math operations below by constraining blade_pitch_hover_rad to be > 0.
    // Assuming 0.1 deg will never be enough to blade pitch angle to maintain hover.
    blade_pitch_hover_rad = MAX(blade_pitch_hover_rad, radians(0.1));

    float b = _param_solidity * _c_l_alpha.get();
    _disc_area = M_PI * 0.25 * sq(_param_diameter);

    // Calculating the equivalent inflow ratio (average across the whole blade)
    float lambda_eq = -b / 16.0 + safe_sqrt(sq(b) / 256.0 + b * blade_pitch_hover_rad / 8.0);

    // Tip speed = head speed (rpm) / 60 * 2pi * rotor diameter/2. Eq below is simplified.
    float tip_speed_auto = _param_head_speed_set_point.get() * M_PI * _param_diameter / 60.0;

    // Calc the coefficient of thrust in the hover
    float c_t_hover = 0.5 * b * (blade_pitch_hover_rad / 3.0 - lambda_eq / 2.0);
    _lift_hover = c_t_hover * 2.0 * SSL_AIR_DENSITY * _disc_area * sq(tip_speed_auto);

    // Estimate rate of descent
    _est_rod = ((0.25 * _param_solidity.get() * ASSUMED_CD0 / c_t_hover) + (sq(c_t_hover) / (_param_solidity * ASSUMED_CD0))) * tip_speed_auto;

    // Estimate rotor C_d
    _c = _lift_hover / sq(_est_rod);

    // Calc flare altitude
    float des_spd_fwd = _param_target_speed * 0.01f;
    calc_flare_alt(-_est_rod, des_spd_fwd);

    // Initialize flare bools
    _flare_complete = false;
    _flare_update_check = false;

    gcs().send_text(MAV_SEVERITY_INFO, "Ct/sigma=%f W=%f kg flare_alt=%f C=%f", c_t_hover/_param_solidity, _lift_hover/GRAVITY_MSS, _flare_alt_calc*0.01f, _c);
    gcs().send_text(MAV_SEVERITY_INFO, "est_rod=%f", _est_rod);
    gcs().send_text(MAV_SEVERITY_INFO, "inflow spd=%f", lambda_eq*tip_speed_auto);
}


void AC_Autorotation::calc_flare_alt(float sink_rate, float fwd_speed)
{
    // Compute speed module and glide path angle during descent
    float speed_module = MAX(norm(sink_rate, fwd_speed), 0.1);
    float glide_angle = M_PI / 2 - safe_asin(fwd_speed / speed_module);

    // Estimate inflow velocity at beginning of flare
    float entry_inflow = - speed_module * sinf(glide_angle + radians(AP_ALPHA_TPP));

    float k_1 = safe_sqrt(_lift_hover / _c);

    // Protect against div by 0 case
    if (is_zero(sink_rate + k_1)) {
        sink_rate -= 0.05;
    }
    if (is_zero(entry_inflow + k_1)) {
        entry_inflow -= 0.05;
    }

    // Estimate flare duration
    float m = _lift_hover / GRAVITY_MSS;
    float k_3 = safe_sqrt((_c * GRAVITY_MSS) / m);
    float k_2 = 1 / (2 * k_3) * logf(fabsf((entry_inflow - k_1)/(entry_inflow + k_1)));
    float a = logf(fabsf((sink_rate - k_1)/(sink_rate + k_1)));
    float b = logf(fabsf((entry_inflow - k_1)/(entry_inflow + k_1)));
    float delta_t_flare = (1 / (2 * k_3)) * (a - b);

    // Estimate flare delta altitude
    float k_4 = (2 * k_2 * k_3) + (2 * k_3 * delta_t_flare);
    float flare_distance;
    flare_distance = ((k_1 / k_3) * (k_4 - logf(fabsf(1-expf(k_4))) - (2 * k_2 * k_3 - logf(fabsf(1 - expf(2 * k_2 * k_3)))))) - k_1 * delta_t_flare;
    float delta_h = -flare_distance * cosf(radians(AP_ALPHA_TPP));

    // Estimate altitude to begin collective pull
    _cushion_alt = (-(sink_rate * cosf(radians(AP_ALPHA_TPP))) * _param_touchdown_time.get()) * 100.0f;

    // Total delta altitude to ground
    _flare_alt_calc = _cushion_alt + delta_h * 100.0f;
}


#if HAL_LOGGING_ENABLED
void AC_Autorotation::Log_Write_Autorotation(void) const
{
    // @LoggerMessage: AROT
    // @Vehicles: Copter
    // @Description: Helicopter AutoRotation information
    // @Field: TimeUS: Time since system startup
    // @Field: hsp: P-term for headspeed controller response
    // @Field: hse: head speed error; difference between current and desired head speed
    // @Field: co: Collective Out
    // @Field: cff: FF-term for headspeed controller response
    // @Field: sf: current forward speed
    // @Field: dsf: desired forward speed
    // @Field: vp: p-term of velocity response
    // @Field: vff: ff-term of velocity response
    // @Field: az: average z acceleration
    // @Field: dvz: Desired Sink Rate
    // @Field: h: height above ground

    //Write to data flash log
    AP::logger().WriteStreaming("AROT",
                                "TimeUS,hsp,hse,co,cff,sf,dsf,vp,vff,az,dvz,h",
                                "Qfffffffffff",
                                AP_HAL::micros64(),
                                _p_term_hs,
                                _head_speed_error,
                                _collective_out,
                                _ff_term_hs,
                                (_speed_forward*0.01f),
                                (_cmd_vel*0.01f),
                                _vel_p,
                                _vel_ff,
                                _avg_acc_z,
                                _desired_sink_rate,
                                (_hagl*0.01f));
}
#endif  // HAL_LOGGING_ENABLED


// Update speed controller
void AC_Autorotation::update_forward_speed_controller(float& pitch_target)
{
    // Specify forward velocity component and determine delta velocity with respect to time
    _speed_forward = calc_speed_forward(); //(cm/s)
    _delta_speed_fwd = _speed_forward - _speed_forward_last; //(cm/s)
    _speed_forward_last = _speed_forward; //(cm/s)

    // Limiting the target velocity based on the max acceleration limit
    if (_cmd_vel < _vel_target) {
        _cmd_vel += _accel_max * _dt;
        if (_cmd_vel > _vel_target) {
            _cmd_vel = _vel_target;
        }
    } else {
        _cmd_vel -= _accel_max * _dt;
        if (_cmd_vel < _vel_target) {
            _cmd_vel = _vel_target;
        }
    }

    // Get p
    _vel_p = _p_fw_vel.get_p(_cmd_vel - _speed_forward);

    // Get ff
    _vel_ff = _cmd_vel * _param_fwd_k_ff;

    // Calculate acceleration target
    _accel_target = _vel_ff + _vel_p;

    // Filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(10.0f);
    _accel_target_filter.apply(_accel_target, _dt);

    // Limits the maximum change in pitch attitude based on acceleration
    if (_accel_target > _accel_out_last + _accel_max) {
        _accel_target = _accel_out_last + _accel_max;
    } else if (_accel_target < _accel_out_last - _accel_max) {
        _accel_target = _accel_out_last - _accel_max;
    }

    // Limiting acceleration based on velocity gained during the previous time step
    if (fabsf(_delta_speed_fwd) > _accel_max * _dt) {
        _flag_limit_accel = true;
    } else {
        _flag_limit_accel = false;
    }

    if ((_flag_limit_accel && fabsf(_accel_target) < fabsf(_accel_out_last)) || !_flag_limit_accel) {
        _accel_out = _accel_target;
    } else {
        _accel_out = _accel_out_last;
    }
    _accel_out_last = _accel_out;

    pitch_target = accel_to_angle(-_accel_out * 0.01) * 100;
}


void AC_Autorotation::update_flare_alt(void)
{
    if (!_flare_update_check) {
        float delta_v_z = fabsf((_inav.get_velocity_z_up_cms()) * 0.01f + _est_rod);

        if ((_speed_forward >= 0.8f * _param_target_speed) && (delta_v_z <= 2) && (fabsf(_avg_acc_z+GRAVITY_MSS) <= 0.5f)) {
            float vel_z = _inav.get_velocity_z_up_cms() * 0.01f;
            float spd_fwd = _speed_forward * 0.01f;
            _c = _lift_hover / sq(vel_z);
            calc_flare_alt(vel_z,spd_fwd);
            _flare_update_check = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Flare_alt_updated=%f C_updated=%f",  _flare_alt_calc*0.01f, _c);
        }
    }
}


// Determine the forward ground speed component from measured components
float AC_Autorotation::calc_speed_forward(void)
{
    auto &ahrs = AP::ahrs();
    Vector2f groundspeed_vector = ahrs.groundspeed_vector();
    float speed_forward = (groundspeed_vector.x*ahrs.cos_yaw() + groundspeed_vector.y * ahrs.sin_yaw()) * 100.0; // (cm/s)
    return speed_forward;
}


// Determine if we are above the touchdown height using the descent rate and param values
bool AC_Autorotation::should_begin_touchdown(void)
{
    float vz = _inav.get_velocity_z_up_cms();

    // We need to be descending for the touch down controller to interpolate the target
    // sensibly between the entry of the touch down phase zero.
    if (vz >= 0.0) {
        return false;
    }

    float time_to_ground = (-_hagl / _inav.get_velocity_z_up_cms());

    return time_to_ground <= _param_touchdown_time.get();
}


void AC_Autorotation::update_avg_acc_z(void)
{
    // Wrap index
    if (_index >= 10) {
        _index = 0;
    }

    _acc_z_sum -= _curr_acc_z[_index];
    _curr_acc_z[_index] = _ahrs.get_accel_ef().z;
    _acc_z_sum += _curr_acc_z[_index];
    _index = _index + 1;

    _avg_acc_z = _acc_z_sum / 10.0;
}
