#include "AC_Autorotation.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AHRS/AP_AHRS.h>

//Autorotation controller defaults
#define AROT_BAIL_OUT_TIME                            2.0f     // Default time for bail out controller to run (unit: s)
#define AROT_FLARE_MIN_ACCEL_PEAK                   1.2f     // Minimum permissible peak acceleration factor for the flare phase (unit: -)
#define AROT_FLARE_TIME_PERIOD_MIN                    0.5f


// Head Speed (HS) controller specific default definitions
#define HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ          2.0f     // low-pass filter on accel error (unit: hz)
#define HS_CONTROLLER_HEADSPEED_P                     0.7f     // Default P gain for head speed controller (unit: -)
#define HS_CONTROLLER_ENTRY_COL_FILTER                0.7f    // Default low pass filter frequency during the entry phase (unit: Hz)
#define HS_CONTROLLER_GLIDE_COL_FILTER                0.1f    // Default low pass filter frequency during the glide phase (unit: Hz)

// Speed Height controller specific default definitions for autorotation use
#define FWD_SPD_CONTROLLER_GND_SPEED_TARGET           1100     // Default target ground speed for speed height controller (unit: cm/s)
#define FWD_SPD_CONTROLLER_MAX_ACCEL                  60      // Default acceleration limit for speed height controller (unit: cm/s/s)
#define AP_FW_VEL_P                       0.9f
#define AP_FW_VEL_FF                      0.15f


const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_Autorotation, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_P
    // @DisplayName: P gain for head spead controller
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
    AP_GROUPINFO("TARG_SP", 4, AC_Autorotation, _param_target_speed, FWD_SPD_CONTROLLER_GND_SPEED_TARGET),

    // @Param: COL_FILT_E
    // @DisplayName: Entry Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the entry phase.  Acts as a following trim.  Must be higher than AROT_COL_FILT_G.
    // @Units: Hz
    // @Range: 0.2 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_E", 5, AC_Autorotation, _param_col_entry_cutoff_freq, HS_CONTROLLER_ENTRY_COL_FILTER),

    // @Param: COL_FILT_G
    // @DisplayName: Glide Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the glide phase.  Acts as a following trim.  Must be lower than AROT_COL_FILT_E.
    // @Units: Hz
    // @Range: 0.03 0.15
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_G", 6, AC_Autorotation, _param_col_glide_cutoff_freq, HS_CONTROLLER_GLIDE_COL_FILTER),

    // @Param: FWD_ACC_MX
    // @DisplayName: Forward Acceleration Limit
    // @Description: Maximum forward acceleration to apply in speed controller.
    // @Units: cm/s/s
    // @Range: 30 60
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("FWD_ACC_MX", 7, AC_Autorotation, _param_accel_max, FWD_SPD_CONTROLLER_MAX_ACCEL),

    // @Param: BAIL_TIME
    // @DisplayName: Bail Out Timer
    // @Description: Time in seconds from bail out initiated to the exit of autorotation flight mode.
    // @Units: s
    // @Range: 0.5 4
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BAIL_TIME", 8, AC_Autorotation, _param_bail_time, AROT_BAIL_OUT_TIME),

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
    // @Description: Velocity (horizontal) P gain.  Determines the propotion of the target acceleration based on the velocity error.
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
    AP_GROUPINFO("FW_V_FF", 11, AC_Autorotation, _param_fwd_k_ff, AP_FW_VEL_FF),

    // @Param: TD_VEL_Z
    // @DisplayName: Desired velocity to initiate the touch down phase
    // @Description: 
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TD_VEL_Z", 12, AC_Autorotation, _param_vel_z_td, 50),

    // @Param: F_PERIOD
    // @DisplayName: Time period to execute the flare
    // @Description: The target time period in which the controller will attempt to complete the flare phase
    // @Units: s
    // @Range: 0.5 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("F_PERIOD", 13, AC_Autorotation, _param_flare_time_period, 0.9),

    // @Param: F_ACC_MAX
    // @DisplayName: Maximum allowable acceleration to be applied by the collective during flare
    // @Description: Multiplier of acceleration due to gravity 'g'.  Cannot be smaller that 1.2.
    // @Range: 1.2 2.5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("F_COL_AC_MX", 14, AC_Autorotation, _param_flare_col_accel_max, 1.5),

    // @Param: TD_ALT_TARG
    // @DisplayName: Target altitude to initiate touch down phase
    // @Description: 
    // @Units: cm
    // @Range: 30 150
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TD_ALT_TARG", 15, AC_Autorotation, _param_td_alt_targ, 50),

    // @Param: LOG
    // @DisplayName: Logging bitmask
    // @Description: 1: Glide phase tuning, 2: Flare phase tuning
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("LOG", 16, AC_Autorotation, _param_log_bitmask, 0),

    // @Param: F_T_RATIO
    // @DisplayName: Time period to execute the flare
    // @Description: The ratio of the time phase that the controller will use to correct miss alignments to the target trajectories
    // @Range: 0.05 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("F_T_RATIO", 17, AC_Autorotation, _param_flare_correction_ratio, 0.1),

    // @Param: COL_FILT_F
    // @DisplayName: Flare Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the flare phase.  Acts as a following trim.
    // @Units: Hz
    // @Range: 0.2 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_F", 18, AC_Autorotation, _param_col_flare_cutoff_freq, 0.8),

    // @Param: COL_F_P
    // @DisplayName: Collective P term for flare controller
    // @Description: 
    // @Range:
    // @Increment:
    // @User: Advanced
    AP_GROUPINFO("COL_F_P", 19, AC_Autorotation, _param_flare_col_p, 3),

    // @Param: ANGLE_MAX
    // @DisplayName: Pitch Angle Limit
    // @Description: The maximum pitch angle (positive or negative) to be applied throughout the autorotation manouver.  If left at zero the 
    // @Units: cdeg
    // @Range: 1000 8000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 20, AC_Autorotation, _param_angle_max, 0),

    // @Param: PIT_F_P
    // @DisplayName: Pitch angle controller p gain for flare phase 
    // @Description: The p gain for the attitude controller during the flare 
    // @Range: 0.1 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PIT_F_P", 21, AC_Autorotation, _param_flare_pitch_p, 3),

    // @Param: PIT_F_FILT
    // @DisplayName: Low pass filter cut off frequency for for pitch angle controller during flare phase 
    // @Description: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("PIT_F_FILT", 22, AC_Autorotation, _param_flare_pitch_cutoff_freq, 0.5),

    AP_GROUPEND
};

// Constructor
AC_Autorotation::AC_Autorotation(AP_InertialNav& inav) :
    _inav(inav),
    _p_hs(HS_CONTROLLER_HEADSPEED_P),
    _p_fw_vel(AP_FW_VEL_P)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

// Initialisation of autorotation controller
void AC_Autorotation::init()
{
    // Reset flags
    _flags.bad_rpm = false;
    _flags.hs_ctrl_running = false;

    // Reset RPM health monitoring
    _unhealthy_rpm_counter = 0;
    _healthy_rpm_counter = 0;

    // Protect against divide by zero
    _param_head_speed_set_point = MAX(_param_head_speed_set_point,500);
    _flare_correction_ratio = MAX(0.05,_param_flare_correction_ratio);
    _flare_time_period = MAX(AROT_FLARE_TIME_PERIOD_MIN,_param_flare_time_period);

    // Get angle max if param set lower than limti
    _angle_max = MAX(_param_angle_max,15); // (cdeg)

    // Ensure forward speed controller acceleration parameter doesn't exceed hard-coded limit
    _accel_max = MIN(_param_accel_max, 60.0f);
}

void AC_Autorotation::init_hs_controller()
{
    // Set head speed controller running flag
    _flags.hs_ctrl_running = true;

    // Set initial collective position to be the collective position on initialisation
    _collective_out = 0.4f;

    // Reset feed forward filter
    col_trim_lpf.reset(_collective_out);
}


bool AC_Autorotation::update_hs_glide_controller(void)
{
    // Reset rpm health flag
    _flags.bad_rpm = false;
    _flags.bad_rpm_warning = false;

    // Get current rpm and update healthly signal counters
    _current_rpm = get_rpm(true);

    if (_unhealthy_rpm_counter <=30) {
        // Normalised head speed
        float head_speed_norm = _current_rpm / _param_head_speed_set_point;

        // Set collective trim low pass filter cut off frequency
        col_trim_lpf.set_cutoff_frequency(_col_cutoff_freq);

        // Calculate the head speed error.  Current rpm is normalised by the set point head speed.  
        // Target head speed is defined as a percentage of the set point.
        _head_speed_error = head_speed_norm - _target_head_speed;

        _p_term_hs = _p_hs.get_p(_head_speed_error);

        // Adjusting collective trim using feed forward (not yet been updated, so this value is the previous time steps collective position)
        _ff_term_hs = col_trim_lpf.apply(_collective_out, _dt);

        // Calculate collective position to be set
        _collective_out = _p_term_hs + _ff_term_hs;

    } else {
        // RPM sensor is bad set collective to minimum
        _collective_out = -1.0f;

        _flags.bad_rpm_warning = true;
    }

    // Send collective setting to motors output library
    set_collective();

    return _flags.bad_rpm_warning;
}


// Function to set collective and collective filter in motor library
void AC_Autorotation::set_collective(void)
{
    AP_Motors *motors = AP::motors();
    if (motors) {
        motors->set_throttle_filter_cutoff(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);
        motors->set_throttle(_collective_out);
    }
}


//function that gets rpm and does rpm signal checking to ensure signal is reliable
//before using it in the controller
float AC_Autorotation::get_rpm(bool update_counter)
{
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    float current_rpm = 0.0f;

    //Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        //Check requested rpm instance to ensure either 0 or 1.  Always defaults to 0.
        if ((_param_rpm_instance > 1) || (_param_rpm_instance < 0)) {
            _param_rpm_instance = 0;
        }

        //Get RPM value
        uint8_t instance = _param_rpm_instance;
        current_rpm = rpm->get_rpm(instance);

        //Check RPM sesnor is returning a healthy status
        if (current_rpm <= -1) {
            //unhealthy, rpm unreliable
            _flags.bad_rpm = true;
        }

    } else {
        _flags.bad_rpm = true;
    }

    if (_flags.bad_rpm) {
        //count unhealthy rpm updates and reset healthy rpm counter
        _unhealthy_rpm_counter++;
        _healthy_rpm_counter = 0;

    } else if (!_flags.bad_rpm && _unhealthy_rpm_counter > 0) {
        //poor rpm reading may have cleared.  Wait 10 update cycles to clear.
        _healthy_rpm_counter++;

        if (_healthy_rpm_counter >= 10) {
            //poor rpm health has cleared, reset counters
            _unhealthy_rpm_counter = 0;
            _healthy_rpm_counter = 0;
        }
    }
    return current_rpm;
}


void AC_Autorotation::log_write_autorotation(void)
{
    // Write logs useful for tuning glide phase
    if (1<<0 & _param_log_bitmask) {
        //Write to data flash log
        AP::logger().Write("AR1G",
                       "TimeUS,P,hserr,ColOut,FFCol,CRPM,SpdF,CmdV,p,ff,AccO,AccT,PitT",
                         "Qffffffffffff",
                        AP_HAL::micros64(),
                        (double)_p_term_hs,
                        (double)_head_speed_error,
                        (double)_collective_out,
                        (double)_ff_term_hs,
                        (double)_current_rpm,
                        (double)calc_speed_forward(),
                        (double)_cmd_vel,
                        (double)_vel_p,
                        (double)_vel_ff,
                        (double)_accel_out,
                        (double)_accel_target,
                        (double)_pitch_target);
    }

    if(1<<1 & _param_log_bitmask){
        //Write to data flash log
        AP::logger().Write("AR2F",
                       "TimeUS,ZAT,AZAT,ZVT,AltT,FAT,AFAT,FP,PitOut,AcMxC,AngMax",
                         "Qffffffffff",
                        AP_HAL::micros64(),
                        (double)_flare_z_accel_targ,
                        (double)_adjusted_z_accel_target,
                        (double)_z_vel_target,
                        (double)_alt_target,
                        (double)_flare_fwd_accel_target,
                        (double)_adjusted_fwd_accel_target,
                        (double)_p_term_pitch,
                        (double)_pitch_out,
                        (double)_flare_resultant_accel_peak,
                        (double)_flare_pitch_ang_max);
    }

}


// Initialise forward speed controller
void AC_Autorotation::init_fwd_spd_controller(void)
{
    // Reset I term and acceleration target  <---TODO: check that there is no longer an I term for this to effect
    _accel_target = 0.0f;

    // Reset cmd vel and last accel to sensible values
    _cmd_vel = calc_speed_forward(); //(cm/s)
    _accel_out_last = _cmd_vel*_param_fwd_k_ff;
}


// set_dt - sets time delta in seconds for all controllers
void AC_Autorotation::set_dt(float delta_sec)
{
    _dt = delta_sec;
}


// update speed controller
void AC_Autorotation::update_forward_speed_controller(void)
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

    // get p
    _vel_p = _p_fw_vel.get_p(_cmd_vel-_speed_forward);

    // get ff
    _vel_ff = _cmd_vel*_param_fwd_k_ff;

    //calculate acceleration target based on PI controller
    _accel_target = _vel_ff + _vel_p;

    // filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(10.0f);
    _accel_target_filter.apply(_accel_target, _dt);

    //Limits the maximum change in pitch attitude based on acceleration
    if (_accel_target > _accel_out_last + _accel_max) {
        _accel_target = _accel_out_last + _accel_max;
    } else if (_accel_target < _accel_out_last - _accel_max) {
        _accel_target = _accel_out_last - _accel_max;
    }

    //Limiting acceleration based on velocity gained during the previous time step 
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

    // update angle targets that will be passed to stabilize controller
    _pitch_target = atanf(-_accel_out/(GRAVITY_MSS * 100.0f))*(18000.0f/M_PI);

}


// Determine the forward ground speed component from measured components
float AC_Autorotation::calc_speed_forward(void)
{
    auto &ahrs = AP::ahrs();
    Vector2f groundspeed_vector = ahrs.groundspeed_vector();
    float speed_forward = (groundspeed_vector.x*ahrs.cos_yaw() + groundspeed_vector.y*ahrs.sin_yaw())* 100; //(cm/s)
    return speed_forward;
}


// Determine whether or not the flare phase should be initiated
bool AC_Autorotation::should_flare(void)
{
    // Measure velocities
    int16_t z_vel = _inav.get_velocity().z;
    int16_t fwd_vel = calc_speed_forward();

    // Measure accelerations
    float z_accel_measure;
    float fwd_accel_measure;
    get_acceleration(z_accel_measure, fwd_accel_measure);

    // Determine peak acceleration if the flare was initiated in this state (cm/s/s)
    _flare_delta_accel_z_peak = 2.0f * (-_param_vel_z_td - z_vel) / _flare_time_period;
    _flare_delta_accel_fwd_peak = 2.0f * (0.0f - fwd_vel) / _flare_time_period;  // Assumed touch down forward speed of 0 m/s

    //Account for gravity in peak z acceleration
    _flare_accel_z_peak = _flare_delta_accel_z_peak + GRAVITY_MSS*100.0f;

    // Account for drag in forward acceleration
    auto &ahrs = AP::ahrs();
    float current_drag = (GRAVITY_MSS * 100.0f - z_accel_measure) * tanf(degrees(ahrs.get_pitch())) - fwd_accel_measure; //(cm/s/s)
    int16_t fwd_vel_prediction = _flare_accel_fwd_peak * _flare_time_period / 4 + fwd_vel;  //this is the approximate velocity at the peak acceleration
    float fwd_accel_delta = abs(_flare_accel_fwd_peak) - current_drag * fwd_vel_prediction * fwd_vel_prediction / (fwd_vel * fwd_vel);  //this is approximately the delta decceleration force required
    fwd_accel_delta = fwd_accel_delta*-1;

    // Resolve the magnitude of the total peak acceleration
    _flare_resultant_accel_peak = sqrtf(_flare_accel_z_peak * _flare_accel_z_peak + fwd_accel_delta * fwd_accel_delta); //(cm/s/s)

    // Compare the calculated peak acceleration to the allowable limits
    if ((_flare_resultant_accel_peak < AROT_FLARE_MIN_ACCEL_PEAK * GRAVITY_MSS * 100.0f)  || (_flare_resultant_accel_peak > _param_flare_col_accel_max * GRAVITY_MSS * 100.0f)){
        //gcs().send_text(MAV_SEVERITY_INFO, "Magnitude Fail");
        return false;
    }

    // Compute the maximum pitch angle
    _flare_pitch_ang_max = acosf(fwd_accel_delta/_flare_resultant_accel_peak)*(18000.0f/M_PI) - 9000.0f;  //(cdeg)

    // Compare the calculated max angle limit to the parameter defined limit
    if (fabsf(_flare_pitch_ang_max) > fabsf(_angle_max)) {
        //gcs().send_text(MAV_SEVERITY_INFO, "Angle Fail");
        return false;
    }

    // Determine the altitude that the flare would complete
    uint32_t td_alt_predicted = 0.237334852f * (_flare_delta_accel_z_peak) * _flare_time_period * _flare_time_period  +  z_vel * _flare_time_period  +  _inav.get_position().z;


        //Write to data flash log
        AP::logger().Write("AFLR",
                       "TimeUS,VZ,VTD,ACC,MIN,MAX,ALT",
                         "Qffffff",
                        AP_HAL::micros64(),
                        (double)_inav.get_velocity().z,
                        (double)_param_vel_z_td,
                        (double)_flare_resultant_accel_peak,
                        (double)(AROT_FLARE_MIN_ACCEL_PEAK * GRAVITY_MSS * 100.0f),
                        (double)(_param_flare_col_accel_max * GRAVITY_MSS * 100.0f),
                        (double)td_alt_predicted);

    // Compare the prediced altitude to the acceptable range
    if ((td_alt_predicted < _param_td_alt_targ * 0.5f)  ||  (td_alt_predicted > _param_td_alt_targ * 1.5f)){
        return false;
    }

    return true;
}


// Set initial conditions for flare targets
void AC_Autorotation::set_flare_initial_cond(void)
{
    _vel_z_initial = _inav.get_velocity().z;
    _vel_fwd_initial = calc_speed_forward();
    _last_vel_z = _vel_z_initial;
    _last_vel_fwd = _vel_fwd_initial;
    _alt_z_initial = _inav.get_position().z;
    _pitch_out = _pitch_target;  //TODO: move flare controller to just continue using pitch out. Dont use pitch_target variable, it is confusing as the controller creates a target which is seperate.

    float z_accel_measure;
    float fwd_accel_measure;
    get_acceleration(z_accel_measure, fwd_accel_measure);

    auto &ahrs = AP::ahrs();  //TODO: get AHRS singleton 

    // Approximate mass normalised drag in forward direction in NED frame using measurements
    _drag_initial = z_accel_measure * tanf(ahrs.get_pitch()) + fwd_accel_measure; //(cm/s/s)
}


/* Use altitude, velocity, and acceleration to come up with a collective position.  Acceleration
can be deamed to have a relationship to collective position.  Use the new collective position to 
feed into head speed controller.  The head speed controller can be used to reverse calculate what 
the target head speed should be.  Compare the head speed target calced from acceleration to the ideal
head speed trajectory to determine an error between desired for kinematics and desired for head speed 
trajectory.  Use energy to weight the coparative difference using a ratio and that will give the new 
target.  That target can then be fed through the actual head speed controller to generate the final output.
This should creat a system whereby the realtive components can be compared using physical relasionships
and won't require PID tuning beyond the original head speed controller.

alt error ==> converted and added to velocity target ==> velocity error ==> converted and added to acceleration 
target ==> acceleration error converted to collective position (linear prediction based on hover col position 
and current col position?)  ==>  Collective position fed through reverse HS controller  ==>  target hs basied on 
accel  ==>  compare that to target head speed from ideal trajectory  ==>  Create energy weighted average target HS
==>  use target HS in HS to collective output controller. */

void AC_Autorotation::init_flare_controller(void)
{
    if (!_flags.hs_ctrl_running){
        _collective_out = 0.5;
        _pitch_target = 0.0f;  //TODO improve this initial based on current pitch angle
    }

    // Set collective and pitch trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_col_cutoff_freq);
    pitch_trim_lpf.set_cutoff_frequency(_param_flare_pitch_cutoff_freq);

    // Reset feed forward filter
    col_trim_lpf.reset(_collective_out);
    pitch_trim_lpf.reset(_pitch_target);
}


void AC_Autorotation::update_flare_controller(void)
{
    // Measure speeds
    int16_t z_vel_measured = _inav.get_velocity().z;
    int16_t fwd_vel_measured = calc_speed_forward();

    //------------------------------------------Targets-------------------------------------------------
    // Calculate the target altitude trajectory
    _alt_target = calc_position_target(_flare_delta_accel_z_peak, _vel_z_initial, _alt_z_initial);


    if (msg_write_once) {
        //gcs().send_text(MAV_SEVERITY_INFO, "_flare_accel_z_peak = %f",_flare_accel_z_peak);
        //gcs().send_text(MAV_SEVERITY_INFO, "_vel_z_initial = %i",_vel_z_initial);
        //gcs().send_text(MAV_SEVERITY_INFO, "_alt_z_initial = %i",_alt_z_initial);
        //gcs().send_text(MAV_SEVERITY_INFO, "_flare_time_period = %f",_flare_time_period);
        //gcs().send_text(MAV_SEVERITY_INFO, "Magnitude Fail");
        msg_write_once = false;
    }





    // Calculate the target velocity trajectories
    _z_vel_target = calc_velocity_target(_flare_delta_accel_z_peak, _vel_z_initial, _alt_target, _inav.get_position().z);
    _fwd_vel_target = calc_velocity_target(_flare_delta_accel_fwd_peak, _vel_fwd_initial);

    // Calculate the target delta acceleration trajectories
    _adjusted_z_accel_target = calc_acceleration_target(_flare_z_accel_targ, _flare_delta_accel_z_peak, _z_vel_target, z_vel_measured);
    //_adjusted_z_accel_target = calc_acceleration_target(_flare_z_accel_targ, _flare_accel_z_peak, 0, 0);
    _adjusted_fwd_accel_target = calc_acceleration_target(_flare_fwd_accel_target, _flare_delta_accel_fwd_peak, _fwd_vel_target, fwd_vel_measured);
    //_adjusted_fwd_accel_target = calc_acceleration_target(_flare_fwd_accel_target, _flare_accel_fwd_peak, 0, 0);

    // Account for gravity
    _total_z_accel_target = _adjusted_z_accel_target +  (GRAVITY_MSS * 100.0f);

    // Account for drag
    float drag = _drag_initial * fwd_vel_measured * fwd_vel_measured / (_vel_fwd_initial * _vel_fwd_initial);
    _total_fwd_accel_target = _adjusted_fwd_accel_target - drag;


    // Calculate target acceleration magnitude
    float flare_accel_mag_target = sqrtf(_total_z_accel_target * _total_z_accel_target + _total_fwd_accel_target * _total_fwd_accel_target);

    // Compute the pitch angle target
    float pitch_ang_target = degrees(acosf(_total_fwd_accel_target/flare_accel_mag_target)) - 90.0f;


    //------------------------------------------Measure-------------------------------------------------
    auto &ahrs = AP::ahrs();

    float z_accel_measured;
    float fwd_accel_measured;
    get_acceleration(z_accel_measured, fwd_accel_measured);

    // Calculate the measured acceleration magnitude
    float flare_accel_mag_measured = sqrtf(z_accel_measured * z_accel_measured + fwd_accel_measured * fwd_accel_measured);


    // Compute the measured pitch angle
    float pitch_ang_measured = degrees(ahrs.get_pitch()); //degrees




    //------------------------------------------Compute Errors-------------------------------------------------
    // Magnitude
    float flare_mag_error = flare_accel_mag_target - flare_accel_mag_measured;

    // Angle
    float pitch_ang_error = pitch_ang_target - pitch_ang_measured;





    // Calculate the p term, based on angle error
    _p_term_pitch = pitch_ang_error * _param_flare_pitch_p;

    // Calculate the p term, based on magnitude error
    _p_term_col = flare_mag_error * _param_flare_col_p / 1000;

    // Adjusting collective trim using feed forward (not yet been updated, so this value is the previous time steps collective position)
    _ff_term_hs = col_trim_lpf.apply(_collective_out, _dt);

    _ff_pitch_term = pitch_trim_lpf.apply(_pitch_out, _dt);


    // Calculate pitch attitude to be set
    _pitch_out = _p_term_pitch + _ff_pitch_term;
    _pitch_out = constrain_float(_pitch_out,-_angle_max,_angle_max);
    _pitch_target = _pitch_out;

    // Calculate the collective position to be set
    _collective_out = _p_term_col + _ff_term_hs;



            //Write to data flash log
        AP::logger().Write("AFLA",
                       "TimeUS,ANGT,ANGM,MAGT,MAGM,DI,DRAG",
                         "Qffffff",
                        AP_HAL::micros64(),
                        (double)pitch_ang_target,
                        (double)pitch_ang_measured,
                        (double)flare_accel_mag_target,
                        (double)flare_accel_mag_measured,
                        (double)_drag_initial,
                        (double)drag);


            //Write to data flash log
        AP::logger().Write("AFLB",
                       "TimeUS,AFAT,TFAT,AZAT,TZAT",
                         "Qffff",
                        AP_HAL::micros64(),
                        (double)_adjusted_fwd_accel_target,
                        (double)_total_fwd_accel_target,
                        (double)_adjusted_z_accel_target,
                        (double)_total_z_accel_target);


            //Write to data flash log
        AP::logger().Write("AFLC",
                       "TimeUS,ALTT,ALTM,AZVT,FVT",
                         "Qffff",
                        AP_HAL::micros64(),
                        (double)_alt_target,
                        (double)_inav.get_position().z,
                        (double)_z_vel_target,
                        (double)_fwd_vel_target);







}

//_alt_target = calc_position_target(_flare_accel_z_peak, _vel_z_initial, _alt_z_initial);
int32_t AC_Autorotation::calc_position_target(float accel_peak, int16_t vel_initial, int32_t pos_initial)
{
    int32_t pos_target = ((accel_peak / 4.0f) * ((_flare_time * _flare_time)   +   ((_flare_time_period * _flare_time_period) / (M_PI * M_2PI)) * (cosf((M_2PI * _flare_time)/_flare_time_period) - 1)))  +  (vel_initial * _flare_time)  +  pos_initial;
    return pos_target;
}


// Overloaded function: Determine the velocity target without altitude correction
int16_t AC_Autorotation::calc_velocity_target(float accel_peak, int16_t vel_initial)
{
    // Calculate the target velocity trajectory
    int16_t vel_target = accel_peak / 2.0f * (_flare_time - _flare_time_period * sinf(_flare_time * M_2PI / _flare_time_period) / M_2PI)  +  vel_initial;
    return vel_target;
}


// Overloaded function: Determine the velocity target with altitude correction
int16_t AC_Autorotation::calc_velocity_target(float accel_peak, int16_t vel_initial, int32_t pos_target, int32_t pos_measured)
{
    // Calculate the target velocity trajectory
    int16_t vel_target = accel_peak / 2.0f * (_flare_time - _flare_time_period * sinf(_flare_time * M_2PI / _flare_time_period) / M_2PI)  +  vel_initial;

    // Calculate velocity correction based on altitude error
    int16_t vel_correction = (pos_target - pos_measured) / (_flare_correction_ratio * _flare_time_period);

    // Adjust velocity target
    int16_t adjusted_vel_target = vel_target - vel_correction;
    return adjusted_vel_target;
}


// Determine the acceleration target and correct target to compensate for velocity error
float AC_Autorotation::calc_acceleration_target(float &accel_target, float accel_peak, int16_t vel_target, int16_t vel_measured)
{
    // Calculate desired acceleration
    accel_target = accel_peak * (1 - cosf((_flare_time * M_2PI)/_flare_time_period)) / 2.0f;

    // Calculate acceleration correction based on velocity error
    float accel_correction = (vel_target - vel_measured) / (_flare_correction_ratio * _flare_time_period);

    // Adjust acceleration target
    float adjusted_accel_target = accel_target + accel_correction;
    return adjusted_accel_target;

}

// Measure accelerations and decompose into vertical and forward directions
void AC_Autorotation::get_acceleration(float& z_accel, float& fwd_accel)
{
    auto &ahrs = AP::ahrs();
    Vector3f accel_ef_blended = ahrs.get_accel_ef_blended(); // (m/s/s)
    z_accel = accel_ef_blended.z*-100.0f; // (cm/s/s) negatice is for change of conventions from NED to the convention used here
    fwd_accel = (accel_ef_blended.x*ahrs.cos_yaw() + accel_ef_blended.y*ahrs.sin_yaw())* 100; // (cm/s/s)
}