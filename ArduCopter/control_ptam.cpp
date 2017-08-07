/*Diego Alberto Mercado Ravell @ die_ravell88@hotmail.com */
//adding altitude control
/*Control flight mode for position control using PTAM
-position and velocity relative to the camera are provided at ptam_pos_vel[6]
-reference roll and pitch angles are used to control x, y position
-z rate is used for altitude
-yaw is set fixed
-if ptam data is not available, the vehicle must try to hove and the pilot can take control (inform the user with the buzzer and leds)
-desired to incorporate optic flow when available!
based on loiter mode*/
#include "Copter.h"
#define ALT_HOLD_ENABLE false

int tiempo=0;
//float ptam_posVel_0[6]={0,0,0,0,0,0};
//float control_alt=0;
float rc_7_norm=1.0,rc_8_norm=1.0, kp_yaw_rate=1000.0;
PtamModeState prev_ptam_state=Ptam_MotorStopped;

/*
 * Init and run calls for ptam flight mode
 */

// ptam_init - initialise ptam controller
bool Copter::ptam_init(bool ignore_checks)
{

    if (PTAM_OK || ignore_checks) {

        // set target to current position
        //wp_nav->init_loiter_target();

            for(int i1=0;i1<3;i1++) {
                    ptam_posVel_0[i1]=ptam_pos_vel[i1];
                    ptam_posVel_0[i1+3]=0;      //target speed is zero
                    ptam_yaw_0=ptam_rpy[2];
            }

        // initialize vertical speed and acceleration
        if(ALT_HOLD_ENABLE){
               pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
            pos_control->set_accel_z(g.pilot_accel_z);
        // initialise position and desired velocity
            if (!pos_control->is_active_z()) {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
            }
        }
        else  {
                if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
                   (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
            }
            // set target altitude to zero for reporting
            pos_control->set_alt_target(0);
        }


hal.uartA->printf("A");
    hal.uartC->printf("PTAM mode on! kp:%f kd:%f\n",(float)g.kp_ptam,(float)g.kd_ptam);
    if((float)RC_Channels::rc_channel(CH_6)->get_control_in()>500){
            rc_7_norm=10.0*(1.0-((float)RC_Channels::rc_channel(CH_7)->get_control_in())/997.0);
            rc_8_norm=10.0*((float)RC_Channels::rc_channel(CH_8)->get_control_in()/997.0);
    }
    hal.uartC->printf("rc 6,7,8:\t%.2f\t%.2f\t%.2f\n\n",(float)RC_Channels::rc_channel(CH_6)->get_control_in(),rc_7_norm,rc_8_norm);


        return true;
    }
    else{
        return false;
    }
}

// _run - runs the  controller
// should be called at 100hz or more
void Copter::ptam_run()
{
    PtamModeState ptam_state;

        //Die map linearly throttle
    ptam_pilot_throttle_scaled = constrain_float(((float)channel_throttle->get_control_in()-100.0)/900.0,0,1);

    float control_y=-PTAM_PD_control(g.kp_ptam*(ptam_pos_vel[0]-ptam_posVel_0[0]),g.kd_ptam*(ptam_pos_vel[3]));
    float control_x=-PTAM_PD_control(g.kp_ptam*(ptam_pos_vel[1]-ptam_posVel_0[1]),g.kd_ptam*(ptam_pos_vel[4]));

    ptam_target_roll=-sin(ptam_rpy[2])*control_x-cos(ptam_rpy[2])*control_y+channel_roll->get_control_in();
    ptam_target_pitch=cos(ptam_rpy[2])*control_x+sin(ptam_rpy[2])*control_y+channel_pitch->get_control_in();
    control_alt=sat_die(-g.kp_ptam_alt*(ptam_pos_vel[2]-ptam_posVel_0[2])-g.kd_ptam_alt*ptam_pos_vel[5],0.15,-0.15);
    control_alt=sat_die( ptam_pilot_throttle_scaled +control_alt,1.0,0.0);


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

   if((float)RC_Channels::rc_channel(CH_6)->get_control_in()>500){
            rc_7_norm=10.0*(1.0-((float)RC_Channels::rc_channel(CH_7)->get_control_in())/997.0);
            rc_8_norm=10.0*((float)RC_Channels::rc_channel(CH_8)->get_control_in())/997.0;
    }

         // get pilot's desired yaw rate
    //ptam_target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    ptam_target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in())+150.0*180.0*(ptam_rpy[2]-ptam_yaw_0)/M_PI;///control yaw with ptam info

    // get pilot's desired throttle, transform pilot's manual throttle input to make hover throttle mid stick, range 0-1
    //ptam_pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());


        if(PTAM_OK){
             attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(ptam_target_roll,ptam_target_pitch,ptam_target_yaw_rate, get_smoothing_gain());
            // output pilot's throttle
            attitude_control->set_throttle_out(control_alt, true, g.throttle_filt);
        }
        else{   //give the pilot full control
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(),channel_pitch->get_control_in(),ptam_target_yaw_rate, get_smoothing_gain());
            // output pilot's throttle
            attitude_control->set_throttle_out(ptam_pilot_throttle_scaled, true, g.throttle_filt);
        }
}

float Copter::PTAM_PD_control(float ptam_pos_err,float ptam_vel_err){
    return sat_die(-g.kp_ptam*(ptam_pos_err)-g.kd_ptam*(ptam_vel_err),MAX_ANGLE_PTAM,-MAX_ANGLE_PTAM);
}

float Copter::sat_die(float d,float A,float a){
    if(A<a){
        float a_t=A;
        A=a;
        a=a_t;
    }
    return d>A?A:(d<a?a:d);
}

