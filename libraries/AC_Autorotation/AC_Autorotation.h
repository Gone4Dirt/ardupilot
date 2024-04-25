#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_MotorsHeli_RSC.h>
#include <AP_Motors/AP_MotorsHeli.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <AC_PID/AC_P.h>
#include <AP_InertialNav/AP_InertialNav.h>  // Inertial Navigation library

class AC_Autorotation
{
public:

    //Constructor
    AC_Autorotation(AP_InertialNav& inav, AP_AHRS& ahrs, AP_MotorsHeli*& motors);

     // object initialisation
    void init();

    // Init and run entry phase controller
    void init_entry(void);
    void run_entry(float& pitch_target);

    // Init and run hover autorotation entry controller
    void init_hover_entry();
    void run_hover_entry(float& pitch_target);

    // Init and run the glide phase controller
    void init_glide(void);
    void run_glide(float& pitch_target);

    // Init and run the flare phase controller
    void init_flare(void);
    void run_flare(float& pitch_target);

    // Init and run the touchdown phase controller
    void init_touchdown(void);
    void run_touchdown(float& pitch_target);

    // Init and run the bailout controller
    void init_bailout(void);
    void run_bailout(float& pitch_target);

    void initial_flare_estimate(void);

    // Update head speed controller
    void update_headspeed_controller(void);

    // Function fetches fresh rpm update and continues sensor health monitoring
    float get_norm_rpm(void);

    float get_bail_time(void) { return _param_bail_time; }

    float get_last_collective() const { return _collective_out; }

    bool is_enable(void) { return _param_enable; }

    void Log_Write_Autorotation(void) const;

    // Update forward speed controller
    void update_forward_speed_controller(float& pitch_target);

    // Calculates the forward speed in the horizontal plane
    float calc_speed_forward(void);

    // set the loop time
    void set_dt(float dt) { _dt = dt; }

    void set_ground_distance(float gnd_dist) { _hagl = (float)gnd_dist; }

    bool above_flare_height(void) const { return _hagl > _flare_alt_calc; }

    // update rolling average z-accel filter
    void update_avg_acc_z(void);

    void update_flare_alt(void);

    void calc_flare_alt(float sink_rate, float fwd_speed);

    // Estimate the time to impact and compare it with the touchdown time param
    bool should_begin_touchdown(void);

    bool use_stabilise_controls(void) const { return int32_t(_options.get()) & int32_t(OPTION::STABILISE_CONTROLS); }

    bool use_autorotation_config(void) const { return int32_t(_options.get()) & int32_t(OPTION::TEST_RANGEFINDER_TIMEOUT); }

    void run_flare_prelim_calc(void);

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];

    static const uint32_t entry_time_ms = 2000;  // (ms) Number of milliseconds that the entry phase operates for

private:

    //--------Internal Variables--------
    float _collective_out;
    float _head_speed_error;         // Error between target head speed and current head speed.  Normalised by head speed set point RPM.
    float _target_head_speed;        // Normalised target head speed.  Normalised by head speed set point RPM.
    float _p_term_hs;                // Proportional contribution to collective setting.
    float _ff_term_hs;               // Following trim feed forward contribution to collective setting.
    float _vel_target;               // Forward velocity target.
    float _pitch_target;             // Pitch angle target.
    float _accel_max;                // Maximum acceleration limit.
    int16_t _speed_forward_last;     // The forward speed calculated in the previous cycle.
    bool _flag_limit_accel;          // Maximum acceleration limit reached flag.
    float _accel_out_last;           // Acceleration value used to calculate pitch target in previous cycle.
    float _cmd_vel;                  // Command velocity, used to get PID values for acceleration calculation.
    float _accel_target;             // Acceleration target, calculated from PID.
    float _delta_speed_fwd;          // Change in forward speed between computation cycles.
    float _dt;                       // Time step.
    int16_t _speed_forward;          // Measured forward speed.
    float _vel_p;                    // Forward velocity P term.
    float _vel_ff;                   // Forward velocity Feed Forward term.
    float _accel_out;                // Acceleration value used to calculate pitch target.
    float _touchdown_init_sink_rate;          // Descent rate at beginning of touvhdown collective pull
    float _touchdown_init_alt;                // Altitude at beginning of touchdown coll pull
    float _flare_entry_speed;        // Traslational velocity at beginning of flare maneuver
    float _desired_speed;            // Desired traslational velocity during flare
    float _desired_sink_rate;        // Desired vertical velocity during touchdown
    float _hagl;                     // Height above ground, passed down from copter, can be from lidar or terrain
    float _avg_acc_z;                // Averaged vertical acceleration
    float _acc_z_sum;                // Sum of vertical acceleration samples
    int16_t _index;                  // Index for vertical acceleration rolling average
    float _curr_acc_z[10];           // Array for storing vertical acceleration samples
    float _flare_alt_calc;           // Calculated flare altitude
    float _lift_hover;               // Main rotor thrust in hover condition
    float _c;                        // Main rotor drag coefficient
    float _cushion_alt;              // Altitude for touchdown collective pull
    float _disc_area;                // Main rotor disc area
    float _est_rod;                  // Estimated rate of descent (vertical autorotation)
    bool  _flare_complete;           // Flare completed
    bool  _flare_update_check;       // Check for flare altitude updating
    uint32_t _time_on_ground;        // Time elapsed after touch down
    uint32_t _last_flare_test_ms;    // Last time the initial flare estimate was run and printed to the GCS

    float _initial_norm_rpm;         // RPM measurement when we first init into autorotation, normalized by set point, used for slewing target of head speed controller
    float _hs_decay;                 // The head speed target acceleration during the entry phase

    LowPassFilterFloat _accel_target_filter; // acceleration target filter

    //--------Parameter Values--------
    AP_Int8  _param_enable;
    AC_P _p_hs;
    AC_P _p_fw_vel;
    AC_P _p_coll_tch;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
    AP_Float _param_col_touchdown_cutoff_freq;
    AP_Int16 _param_accel_max;
    AP_Float _param_bail_time;
    AP_Int8  _param_rpm_instance;
    AP_Float _param_fwd_k_ff;
    AP_Int16 _param_target_speed;
    AP_Int16 _param_head_speed_set_point;
    AP_Float _param_solidity;
    AP_Float _param_diameter;
    AP_Float _param_touchdown_time;
    AP_Int32 _options;
    AP_Float _c_l_alpha;

    enum class OPTION {
        STABILISE_CONTROLS = (1<<0),
        PRINT_FLARE_ESTIMATES = (1<<1),
        TEST_RANGEFINDER_TIMEOUT = (1<<2),
    };

    // low pass filter for collective trim
    LowPassFilterFloat col_trim_lpf;

    //--------References to Other Libraries--------
    AP_InertialNav&    _inav;
    AP_AHRS&           _ahrs;
    AP_MotorsHeli*&    _motors_heli;
};
