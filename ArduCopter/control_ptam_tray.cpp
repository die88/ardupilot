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
//#define ALT_HOLD_ENABLE false


/*
 * Init and run calls for ptam flight mode
 */

// ptam_init - initialise ptam controller
bool Copter::ptam_tray_init(bool ignore_checks)
{

    if (PTAM_OK || ignore_checks) {

        // set target to current position
        //wp_nav->init_loiter_target();

        ptam_tray_mode=Ptam_tray_Home;
        wp_index=0;

            for(int i1=0;i1<3;i1++) {
                    ptam_posVel_0[i1]=ptam_pos_vel[i1];
                    ptam_posVel_0[i1+3]=0;      //target speed is zero
                    ptam_pos_target[i1]=ptam_pos_vel[i1];
                    ptam_pos_target[i1+3]=0;      //target speed is zero
                    ptam_yaw_0=ptam_rpy[2];
                    for(int i2=0;i2<WP_NUM;i2++){
                        ptam_wp[i1][i2]=ptam_posVel_0[i1];
                        ptam_wp[i1+3][i2]=0;      //target speed is zero
                    }
                }

        //define the waypoints
            //sweep a wall
        ptam_wp[0][0]+=1.0; //go 1m in x
        ptam_wp[0][1]-=1.0; //go -2m in x and 0.5 up
        ptam_wp[2][1]+=0.5;
        ptam_wp[0][2]+=1.0; //go 2m in x
        ptam_wp[2][2]+=0.5;
        ptam_wp[0][3]-=1.0; //go -2m in x 0.5 up
        ptam_wp[2][3]+=1.0;
        ptam_wp[0][4]+=1.0; //go 2m in x
        ptam_wp[2][4]+=1.0;

        // initialize vertical speed and acceleration

                if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
                   (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
                            return false;
            }
            // set target altitude to zero for reporting
            pos_control->set_alt_target(0);


hal.uartA->printf("A");
    hal.uartC->printf("PTAM_tray mode on! kp:%f kd:%f\n",(float)g.kp_ptam,(float)g.kd_ptam);
    if((float)RC_Channels::rc_channel(CH_6)->get_control_in()>500){
            rc_7_norm=10.0*(1.0-((float)RC_Channels::rc_channel(CH_7)->get_control_in())/997.0);
            rc_8_norm=10.0*((float)RC_Channels::rc_channel(CH_8)->get_control_in()/997.0);
    }
    hal.uartC->printf("rc 6,7,8:\t%.2f\t%.2f\t%.2f\n\n",(float)RC_Channels::rc_channel(CH_6)->get_control_in(),rc_7_norm,rc_8_norm);


        return true;
    }
    else{
            gcs().send_text(MAV_SEVERITY_WARNING,"PTAM off!");
        return false;
    }
}

// _run - runs the  controller
// should be called at 100hz or more
void Copter::ptam_tray_run()
{

        //Die map linearly throttle
    ptam_pilot_throttle_scaled = constrain_float(((float)channel_throttle->get_control_in()-100.0)/900.0,0,1);

    float control_y=-PTAM_PD_control(g.kp_ptam*(ptam_pos_vel[0]-ptam_pos_target[0]),g.kd_ptam*(ptam_pos_vel[3]));   //missing target speed, zero by now
    float control_x=-PTAM_PD_control(g.kp_ptam*(ptam_pos_vel[1]-ptam_pos_target[1]),g.kd_ptam*(ptam_pos_vel[4]));   //missing target speed, zero by now

    ptam_target_roll=-sin(ptam_rpy[2])*control_x-cos(ptam_rpy[2])*control_y+channel_roll->get_control_in();
    ptam_target_pitch=cos(ptam_rpy[2])*control_x+sin(ptam_rpy[2])*control_y+channel_pitch->get_control_in();
    control_alt=sat_die(-g.kp_ptam_alt*(ptam_pos_vel[2]-ptam_pos_target[2])-g.kd_ptam_alt*ptam_pos_vel[5],0.15,-0.15); //missing target speed, zero by now
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
    ptam_target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in())+constrain_int32(150.0*180.0*(ptam_rpy[2]-ptam_yaw_0)/M_PI,-3000,3000);///control yaw with ptam info

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


