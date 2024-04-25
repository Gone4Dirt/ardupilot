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

#define BAILOUT_MOTOR_RAMP_TIME        1.0f    // (s) Time set on bailout ramp up timer for motors - See AC_MotorsHeli_Single

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

    // Init autorotation controllers object
    g2.arot.init();

    // Display message
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");

    // Check if we have sufficient speed or height to do a full autorotation otherwise we have to do one from the hover
    // Note: This must be called after arot.init()
    _hover_autorotation = (inertial_nav.get_speed_xy_cms() < 250.0f) && !g2.arot.above_flare_height();

    // Setting default starting switches
    phase_switch = Autorotation_Phase::ENTRY;

    // Set all phase init flags
    _flags.entry_init = false;
    _flags.hover_entry_init = false;
    _flags.ss_glide_init = false;
    _flags.flare_init = false;
    _flags.touch_down_init = false;
    _flags.bail_out_init = false;

    // Set entry timer
    _entry_time_start_ms = millis();

    return true;
}


void ModeAutorotate::run()
{
    // Current time
    uint32_t now = millis(); //milliseconds

    // Set dt in library
    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    g2.arot.set_dt(last_loop_time_s);

    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------
    // State machine progresses through the autorotation phases as you read down through the if statements.
    // More urgent phases (the ones closer to the ground) take precedence later in the if statements. Init
    // flags are used to prevent flight phase regression

    if (!_hover_autorotation && !_flags.ss_glide_init && g2.arot.above_flare_height() && ((now - _entry_time_start_ms) > g2.arot.entry_time_ms)) {
        // Flight phase can be progressed to steady state glide
        phase_switch = Autorotation_Phase::SS_GLIDE;
    }

    // Check if we are between the flare start height and the touchdown height
    if (!_hover_autorotation && !_flags.flare_init && !g2.arot.above_flare_height() && !g2.arot.should_begin_touchdown()) {
        phase_switch = Autorotation_Phase::FLARE;
    }

    // Initial check to see if we need to perform a hover autorotation
    if (_hover_autorotation && !_flags.hover_entry_init) {
        phase_switch = Autorotation_Phase::HOVER_ENTRY;
    }

    // Begin touch down if within touch down time
    if (!_flags.touch_down_init && g2.arot.should_begin_touchdown()) {
        phase_switch = Autorotation_Phase::TOUCH_DOWN;
    }

    // Check if interlock becomes engaged
    if (motors->get_interlock() && !copter.ap.land_complete) {
        phase_switch = Autorotation_Phase::BAIL_OUT;
    } else if (motors->get_interlock() && copter.ap.land_complete) {
        // Aircraft is landed and no need to bail out
        set_mode(copter.prev_control_mode, ModeReason::AUTOROTATION_BAILOUT);
    }


    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    float pitch_target = 0.0;

    // Always keep accel filter up to date in autorotation lib
    g2.arot.update_avg_acc_z();

    // Run appropriate controller based on flight phase
    switch (phase_switch) {

        case Autorotation_Phase::ENTRY: {
            // Smoothly transition the collective to enter autorotation regime and either build or maintain forward speed
            if (!_flags.entry_init) {
                g2.arot.init_entry();
                _flags.entry_init = true;
            }
            g2.arot.run_entry(pitch_target);
            break;
        }
        case Autorotation_Phase::HOVER_ENTRY: {
            // Controller phase where the aircraft is too low and too slow to perform a full autorotation
            // instead, we will try to minimize rotor drag until we can jump to the touch down phase
            if (!_flags.hover_entry_init) {
                g2.arot.init_hover_entry();
                _flags.hover_entry_init = true;
            }
            g2.arot.run_hover_entry(pitch_target);
            break;
        }

        case Autorotation_Phase::SS_GLIDE: {
            // Maintain head speed and forward speed as we glide to the ground
            if (!_flags.ss_glide_init) {
                g2.arot.init_glide();
                _flags.ss_glide_init = true;
            }
            g2.arot.run_glide(pitch_target);
            break;
        }

        case Autorotation_Phase::FLARE: {
            // Smoothly slow the aircraft to a stop by pitching up, maintaining set point head speed throughout.
            if (!_flags.flare_init) {
                g2.arot.init_flare();
                _flags.flare_init = true;
            }
            g2.arot.run_flare(pitch_target);
            break;
        }

        case Autorotation_Phase::TOUCH_DOWN: {
            // Ensure vehicle is level and use energy stored in head to gently touch down on the ground
            if (!_flags.touch_down_init) {
                g2.arot.init_touchdown();
                _flags.touch_down_init = true;
            }

            // Run touchdown controller
            g2.arot.run_touchdown(pitch_target);
            break;
        }

        case Autorotation_Phase::BAIL_OUT: {

            float curr_vel_z = inertial_nav.get_velocity_z_up_cms();

            if (!_flags.bail_out_init) {
                // Functions and settings to be done once are done here.
                gcs().send_text(MAV_SEVERITY_INFO, "Bailing Out of Autorotation");

                // Set bail out timer remaining equal to the parameter value, bailout time
                // cannot be less than the motor spool-up time: BAILOUT_MOTOR_RAMP_TIME.
                _bail_time = MAX(g2.arot.get_bail_time(),BAILOUT_MOTOR_RAMP_TIME+0.1f);

                // Set bail out start time
                _bail_time_start_ms = now;

                // Set initial target vertical speed
                _desired_v_z = curr_vel_z;

                // Initialise position and desired velocity
                if (!pos_control->is_active_z()) {
                    pos_control->relax_z_controller(motors->get_throttle());
                }

                // Get pilot parameter limits
                const float pilot_spd_up = g.pilot_speed_up;

                float pilot_des_v_z = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
                // Don't allow the pilot to request positive climb when slewing to target climb rate on bailout.
                // It is easy to leave the throttle stick high by accident
                pilot_des_v_z = MIN(0, pilot_des_v_z);

                // Calculate target climb rate adjustment to transition from bail out descent speed to requested climb rate on stick.
                _target_climb_rate_adjust = (curr_vel_z - pilot_des_v_z)/(_bail_time - BAILOUT_MOTOR_RAMP_TIME); //accounting for 0.5s motor spool time

                // Calculate pitch target adjustment rate to return to level
                _target_pitch_adjust = ahrs.get_pitch()/_bail_time;

                // set vertical speed and acceleration limits
                pos_control->set_max_speed_accel_z(curr_vel_z, pilot_spd_up, fabsf(_target_climb_rate_adjust));
                pos_control->set_correction_speed_accel_z(curr_vel_z, pilot_spd_up, fabsf(_target_climb_rate_adjust));

                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

                _flags.bail_out_init = true;
            }

            if ((now - _bail_time_start_ms)*1e-3 >= BAILOUT_MOTOR_RAMP_TIME) {
                // Update desired vertical speed and pitch target after the bailout motor ramp timer has completed
                _desired_v_z -= _target_climb_rate_adjust * last_loop_time_s;
                pitch_target -= _target_pitch_adjust * last_loop_time_s;
            }

            // Set position controller
            pos_control->set_pos_target_z_from_climb_rate_cm(_desired_v_z);

            // Update controllers
            pos_control->update_z_controller();

            if ((now - _bail_time_start_ms)*1e-3 >= _bail_time) {
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

    //----------------------------------------------------------------
    //                  Navigation Options
    //----------------------------------------------------------------

    float pilot_roll, pilot_pitch, pilot_yaw_rate, target_roll;
    // Operator is in control of roll and yaw. Pitch is controlled by speed-height controller.
    get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // Allow pilot control of the roll and yaw. Two control schemes available, a stabilise-like control scheme
    // and a loiter like control scheme.
    if (g2.arot.use_stabilise_controls()) {
        // Get pilot's desired yaw rate
        pilot_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        target_roll = pilot_roll;

    } else {
        // loiter-like control scheme
        // rotate roll, pitch input from north facing to vehicle's perspective
        Vector2f vel = ahrs.earth_to_body2D(inertial_nav.get_velocity_neu_cms().xy());
        float roll_vel = vel.y;
        float pitch_vel = vel.x;

        // gain scheduling for yaw
        // TODO: Need to change consts to sensibly named defines
        float pitch_vel2 = MIN(fabsf(pitch_vel), 2000);
        pilot_yaw_rate = ((float)pilot_roll/1.0f) * (1.0f - (pitch_vel2 / 5000.0f)) * g2.command_model_pilot.get_rate() / 45.0;

        roll_vel = constrain_float(roll_vel, -560.0f, 560.0f);
        pitch_vel = constrain_float(pitch_vel, -560.0f, 560.0f);

        // convert user input into desired roll velocity
        float roll_vel_error = roll_vel - (pilot_roll / 0.8f);

        // roll velocity is feed into roll acceleration to minimize slip
        target_roll = roll_vel_error * -0.8f;
        target_roll = constrain_float(target_roll, -4500.0f, 4500.0f);
    }

    // Pitch target is calculated in autorotation phase switch above
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, pitch_target, pilot_yaw_rate);

}


void ModeAutorotate::exit(void) {
    copter.reset_rangefinder_timeout();
}

#endif
