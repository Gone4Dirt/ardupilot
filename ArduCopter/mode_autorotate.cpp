/* -----------------------------------------------------------------------------------------
    This is currently a SITL only function until the project is complete.
    To trial this in SITL you will need to use Real Flight 8.
    Instructions for how to set this up in SITL can be found here:
    https://discuss.ardupilot.org/t/autonomous-autorotation-gsoc-project-blog/42139
 -----------------------------------------------------------------------------------------*/

#include "Copter.h"
#include <AC_Autorotation/AC_Autorotation.h>
#include "mode.h"

#include <utility>

#if MODE_AUTOROTATE_ENABLED == ENABLED

#define AUTOROTATE_ENTRY_TIME          2.0f    // (s) number of seconds that the entry phase operates for
#define BAILOUT_MOTOR_RAMP_TIME        1.0f    // (s) time set on bailout ramp up timer for motors - See AC_MotorsHeli_Single
#define HEAD_SPEED_TARGET_RATIO        1.0f    // Normalised target main rotor head speed (unit: -)

bool ModeAutorotate::init(bool ignore_checks)
{
#if FRAME_CONFIG != HELI_FRAME
    // Only allow trad heli to use autorotation mode
    return false;
#endif

    // Check that mode is enabled
    if (!g2.arot.is_enable()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Autorot Mode Not Enabled");
        return false;
    }

    // Check that interlock is disengaged
    if (motors->get_interlock()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Autorot Mode Change Fail: Interlock Engaged");
        return false;
    }

    // Initialise controllers
    // This must be done before RPM value is fetched
    g2.arot.init_hs_controller();
    g2.arot.init_fwd_spd_controller();

    // Retrive rpm and start rpm sensor health checks
    _initial_rpm = g2.arot.get_rpm(true);

    // Display message 
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");

     // Set all initial flags to on
    _flags.entry_initial = 1;
    _flags.ss_glide_initial = 1;
    _flags.flare_initial = 1;
    _flags.touch_down_initial = 1;
    _flags.level_initial = 1;
    _flags.break_initial = 1;
    _flags.straight_ahead_initial = 1;
    _flags.bail_out_initial = 1;
    _msg_flags.bad_rpm = true;
    _msg_flags.flare_exit_td_alt = true;
    _msg_flags.flare_exit_timer = true;

    // Setting default starting switches
    phase_switch = Autorotation_Phase::ENTRY;

    // Set entry timer
    _entry_time_start = millis();

    // The decay rate to reduce the head speed from the current to the target
    _hs_decay = ((_initial_rpm/g2.arot.get_hs_set_point()) - HEAD_SPEED_TARGET_RATIO) / AUTOROTATE_ENTRY_TIME;

    return true;
}



void ModeAutorotate::run()
{
    // Check if interlock becomes engaged
    if (motors->get_interlock() && !copter.ap.land_complete) {
        phase_switch = Autorotation_Phase::BAIL_OUT;
    } else if (motors->get_interlock() && copter.ap.land_complete) {
        // Aircraft is landed and no need to bail out
        set_mode(copter.prev_control_mode, ModeReason::AUTOROTATION_BAILOUT);
    }

    // Current time
    now = AP_HAL::millis(); //milliseconds

    // Initialise internal variables
    float curr_vel_z = inertial_nav.get_velocity().z;   // Current vertical descent
    int32_t curr_alt = inertial_nav.get_position().z;

    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------

     // Setting default phase switch positions
     nav_pos_switch = Navigation_Decision::USER_CONTROL_STABILISED;

    // Timer from entry phase to progress to glide phase
    if (phase_switch == Autorotation_Phase::ENTRY){

        if ((now - _entry_time_start)/1000.0f > AUTOROTATE_ENTRY_TIME) {
            // Flight phase can be progressed to steady state glide
            phase_switch = Autorotation_Phase::SS_GLIDE;
        }
    }

    // Check for flare exit conditions
    if (phase_switch != Autorotation_Phase::TOUCH_DOWN  &&  phase_switch != Autorotation_Phase::BAIL_OUT  &&  g2.arot.get_td_alt_targ() >= curr_alt) {
            phase_switch = Autorotation_Phase::TOUCH_DOWN;
            message_handler(Msg_Num::FLARE_EXIT_ALT);
    }
    if (phase_switch == Autorotation_Phase::FLARE){
        // This must be nested to recall sensible value of _flare_time_start
        if ((_flare_time_start - now)/1000.0f >= g2.arot.get_flare_time_period()) {
            phase_switch = Autorotation_Phase::TOUCH_DOWN;
            message_handler(Msg_Num::FLARE_EXIT_TIMER);
        }
    }

    // Check for flare initiation conditions
    if (phase_switch != Autorotation_Phase::FLARE  &&  phase_switch != Autorotation_Phase::TOUCH_DOWN){
        // should_flare function must be called within the nested if to prevend peak accel from 
        // being updated once the flare is initiated
        if (g2.arot.should_flare()){
            phase_switch = Autorotation_Phase::FLARE;
        }
    }


    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    switch (phase_switch) {

        case Autorotation_Phase::ENTRY:
        {
            // Entry phase functions to be run only once
            if (_flags.entry_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");
                #endif

                // Set following trim low pass cut off frequency
                g2.arot.set_col_cutoff_freq(g2.arot.get_col_entry_freq());

                // Target head speed is set to rpm at initiation to prevent unwanted changes in attitude
                _target_head_speed = _initial_rpm/g2.arot.get_hs_set_point();

                // Set desired forward speed target
                g2.arot.set_desired_fwd_speed();

                // Prevent running the initial entry functions again
                _flags.entry_initial = 0;

            }

            // Slowly change the target head speed until the target head speed matches the parameter defined value
            if (g2.arot.get_rpm() > HEAD_SPEED_TARGET_RATIO*1.005f  ||  g2.arot.get_rpm() < HEAD_SPEED_TARGET_RATIO*0.995f) {
                _target_head_speed -= _hs_decay*G_Dt;
            } else {
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;
            }

            // Set target head speed in head speed controller
            g2.arot.set_target_head_speed(_target_head_speed);

            // Update time step
            g2.arot.set_dt(G_Dt);

            // Run airspeed/attitude controller
            g2.arot.update_forward_speed_controller();

            // Retrieve pitch target
            _pitch_target = g2.arot.get_pitch();

            // Update controllers
            _flags.bad_rpm = g2.arot.update_hs_glide_controller(); //run head speed/ collective controller

            break;
        }

        case Autorotation_Phase::SS_GLIDE:
        {
            // Steady state glide functions to be run only once
            if (_flags.ss_glide_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "SS Glide Phase");
                #endif

                // Set following trim low pass cut off frequency
                g2.arot.set_col_cutoff_freq(g2.arot.get_col_glide_freq());

                // Set desired forward speed target
                g2.arot.set_desired_fwd_speed();

                // Set target head speed in head speed controller
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;  //Ensure target hs is set to glide incase hs hasent reached target for glide
                g2.arot.set_target_head_speed(_target_head_speed);

                // Prevent running the initial glide functions again
                _flags.ss_glide_initial = 0;
            }

            // Update time step
            g2.arot.set_dt(G_Dt);

            // Run airspeed/attitude controller
            g2.arot.update_forward_speed_controller();

            // Retrieve pitch target 
            _pitch_target = g2.arot.get_pitch();

            // Update head speed/ collective controller
            _flags.bad_rpm = g2.arot.update_hs_glide_controller(); 
            // Attitude controller is updated in navigation switch-case statements

            break;
        }

        case Autorotation_Phase::FLARE:
        {
             // Flare phase functions to be run only once
            if (_flags.flare_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Flare Phase");
                #endif

                // Set flare initiate time
                _flare_time_start = now;

                // Set initial conditions in controller
                g2.arot.set_flare_initial_cond();

                // Set following trim low pass cut off frequency
                g2.arot.set_col_cutoff_freq(g2.arot.get_col_flare_freq());

                // Prevent running the initial flare functions again
                _flags.flare_initial = 0;
            }

            // Set flare time in controller
            g2.arot.set_flare_time(now - _flare_time_start);

            // Update time step
            g2.arot.set_dt(G_Dt);

            // Set target head speed in head speed controller
            g2.arot.set_target_head_speed(HEAD_SPEED_TARGET_RATIO);

            // Update head speed/ collective controller
            //_flags.bad_rpm = g2.arot.update_hs_glide_controller(); 

            // Calculate new head speed target based on positional trajectory
            _pitch_target = g2.arot.update_flare_controller();
            // Attitude controller is updated in navigation switch-case statements

            break;
        }

        case Autorotation_Phase::TOUCH_DOWN:
        {
            // Touch down phase functions to be run only once
            if (_flags.touch_down_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Touch Down Phase");
                #endif

                // Initialise position and desired velocity
                if (!pos_control->is_active_z()) {
                    pos_control->relax_alt_hold_controllers(g2.arot.get_last_collective());
                }

                // Set acceleration limit
                int16_t end_vel = 50;
                int32_t accel_linear = (end_vel * end_vel  -  curr_vel_z * curr_vel_z) / (2 * curr_alt);
                pos_control->set_max_accel_z(abs(accel_linear));

                // Set speed limit
                pos_control->set_max_speed_z(curr_vel_z, 0.0f);

                _flags.touch_down_initial = 0;
            }

            // Set position controller
            pos_control->set_alt_target_from_climb_rate(abs(g2.arot.get_td_vel_targ())*-1, G_Dt, true);

            // Update controllers
            pos_control->update_z_controller();


            _pitch_target = 0.0f;

            break;
        }

        case Autorotation_Phase::BAIL_OUT:
        {
        if (_flags.bail_out_initial == 1) {
                // Functions and settings to be done once are done here.

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Bailing Out of Autorotation");
                #endif

                // Set bail out timer remaining equal to the paramter value, bailout time 
                // cannot be less than the motor spool-up time: BAILOUT_MOTOR_RAMP_TIME.
                _bail_time = MAX(g2.arot.get_bail_time(),BAILOUT_MOTOR_RAMP_TIME+0.1f);

                // Set bail out start time
                _bail_time_start = now;

                // Set initial target vertical speed
                _desired_v_z = curr_vel_z;

                // Initialise position and desired velocity
                if (!pos_control->is_active_z()) {
                    pos_control->relax_alt_hold_controllers(g2.arot.get_last_collective());
                }

                // Get pilot parameter limits
                const float pilot_spd_dn = -get_pilot_speed_dn();
                const float pilot_spd_up = g.pilot_speed_up;

                // Set speed limit
                pos_control->set_max_speed_z(curr_vel_z, pilot_spd_up);

                float pilot_des_v_z = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
                pilot_des_v_z = constrain_float(pilot_des_v_z, pilot_spd_dn, pilot_spd_up);

                // Calculate target climb rate adjustment to transition from bail out decent speed to requested climb rate on stick.
                _target_climb_rate_adjust = (curr_vel_z - pilot_des_v_z)/(_bail_time - BAILOUT_MOTOR_RAMP_TIME); //accounting for 0.5s motor spool time

                // Calculate pitch target adjustment rate to return to level
                _target_pitch_adjust = _pitch_target/_bail_time;

                // Set acceleration limit
                pos_control->set_max_accel_z(abs(_target_climb_rate_adjust));

                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

                _flags.bail_out_initial = 0;
            }

        if ((now - _bail_time_start)/1000.0f >= BAILOUT_MOTOR_RAMP_TIME) {
            // Update desired vertical speed and pitch target after the bailout motor ramp timer has completed
            _desired_v_z -= _target_climb_rate_adjust*G_Dt;
            _pitch_target -= _target_pitch_adjust*G_Dt;
        }
        // Set position controller
        pos_control->set_alt_target_from_climb_rate(_desired_v_z, G_Dt, false);

        // Update controllers
        pos_control->update_z_controller();

        if ((now - _bail_time_start)/1000.0f >= _bail_time) {
            // Bail out timer complete.  Change flight mode. Do not revert back to auto. Prevent aircraft
            // from continuing mission and potentially flying further away after a power failure.
            if (copter.prev_control_mode == Mode::Number::AUTO) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::AUTOROTATION_BAILOUT);
            } else {
                set_mode(copter.prev_control_mode, ModeReason::AUTOROTATION_BAILOUT);
            }
        }

        break;
        }
    }


    switch (nav_pos_switch) {

        case Navigation_Decision::USER_CONTROL_STABILISED:
        {
            // Operator is in control of roll and yaw.  Controls act as if in stabilise flight mode.  Pitch 
            // is controlled by speed-height controller.
            float pilot_roll, pilot_pitch;
            get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

            // Get pilot's desired yaw rate
            float pilot_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

            // Pitch target is calculated in autorotation phase switch above
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, _pitch_target, pilot_yaw_rate);
            break;
        }

        case Navigation_Decision::STRAIGHT_AHEAD:
        case Navigation_Decision::INTO_WIND:
        case Navigation_Decision::NEAREST_RALLY:
        {
            break;
        }
    }

    // Output warning messaged if rpm signal is bad
    if (_flags.bad_rpm) {
        message_handler(Msg_Num::POOR_RPM_SENSOR);
    }




//Write to data flash log
//    AP::logger().Write("ARO3",
 //                      "TimeUS,AZ",
  //                       "Qf",
   //                     AP_HAL::micros64(),
    //                    (double) g2.arot.update_flare_controller());









} // End function run()

void ModeAutorotate::message_handler(Msg_Num message_n)
{
    switch (message_n) {
        case Msg_Num::POOR_RPM_SENSOR:
        {
            if (_msg_flags.bad_rpm) {
                // Bad rpm sensor health.
                gcs().send_text(MAV_SEVERITY_INFO, "Warning: Poor RPM Sensor Health");
                gcs().send_text(MAV_SEVERITY_INFO, "Action: Minimum Collective Applied");
                _msg_flags.bad_rpm = false;
            }
            break;
        }

        case Msg_Num::FLARE_EXIT_ALT:
        {
            if (_msg_flags.flare_exit_td_alt) {
                // Userful to know for tuning whether the Touch down phase was initiated due to 
                gcs().send_text(MAV_SEVERITY_INFO, "TD Reason Alt");
                _msg_flags.flare_exit_td_alt = false;
            }
            break;
        }

        case Msg_Num::FLARE_EXIT_TIMER:
        {
            if (_msg_flags.flare_exit_td_alt) {
                // Userful to know for tuning whether the Touch down phase was initiated due to 
                gcs().send_text(MAV_SEVERITY_INFO, "TD Reason Timer");
                _msg_flags.flare_exit_timer = false;
            }
            break;
        }
    }
}

#endif
