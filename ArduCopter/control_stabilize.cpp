#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

 int contador=0;

//const AP_HAL::HAL& hal_uartE = AP_HAL::get_HAL();

// stabilize_init - initialise stabilize controller
bool Copter::stabilize_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

        // initialize uartE
        //hal.uartE->begin(115200);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::stabilize_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    pilot_throttle_scaled = constrain_float(((float)channel_throttle->get_control_in()-100.0)/900.0,0,1);//Die linearly map throttle
     control_alt=pilot_throttle_scaled;

       /* if(contador++==100){
        contador=0;

        //hal.uartE->printf("hola amiguitos! uartE at %.3f seconds\n",(double)(AP_HAL::millis()*0.001f));
    }*/

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    //pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());
    pilot_throttle_scaled = constrain_float(((float)channel_throttle->get_control_in()-100.0)/900.0,0,1);//Die linearly map throttle

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
