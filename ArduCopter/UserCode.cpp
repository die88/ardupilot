//Diego Alberto Mercado Ravell

#include "Copter.h"
#define CMD_LEN 23
#define MAX_DIST 20
#define DIST_TARGET 0.2

int contador_1=0,i_2=0,contador_2=0;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

//#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    //Diego recovering raw IMU measurements
    //AP_InertialSensor my_ins= ahrs.get_ins();
    //Vector3f accs= my_ins.get_accel();
    //Vector3f gyros= my_ins.get_gyro();

    // put your 100Hz code here
//    if(contador_1++==500){
//            contador_1=0;
//            hal.uartA->print("a");
//            //hal.uartE->print("e");
//    }

    int buff_len = hal.uartE->available();
    for(int i_1 = 0; i_1<buff_len; i_1++){
       int16_t uart_byte=hal.uartE->read();
        //hal.uartA->print('r');
         //hal.uartA->print((char) uart_byte);
       int8_t pack_complete=readPTAM((unsigned char)uart_byte);
       if(pack_complete!=0){
            //hal.uartA->printf("complete!: ");
        if(packPos[1]=='o') {
                PTAM_OK=true;
            //hal.uartA->printf("pos, vel, rpy:%.2f %.2f %.2f\t%.2f %.2f %.2f\t%.2f %.2f %.2f\n",ptam_pos_vel[0],ptam_pos_vel[1],ptam_pos_vel[2],ptam_pos_vel[3],ptam_pos_vel[4],ptam_pos_vel[5],ptam_rpy[0]*180/M_PI,ptam_rpy[1]*180/M_PI,ptam_rpy[2]*180/M_PI);
        }
        else {
            PTAM_OK=false;
            hal.uartA->printf("error!: ");
            if(packPos[1]=='e')  hal.uartA->printf("no new packages\n");
            }
        }
    }

        if(i_2++==20){
            plot_ptam();
            //hal.uartC->printf("yaw real: %.2f, ptam: %.2f, ptam_0: %.2f\n",ahrs.yaw,ptam_rpy[2],ptam_yaw_0); //just to debug
            //float ptam_yaw_rate_pilot =  get_pilot_desired_yaw_rate(channel_yaw->get_control_in())+500.0*180.0*(ptam_rpy[2]-ptam_yaw_0)/M_PI;//control yaw with ptam info
            //hal.uartC->printf("pilot yaw rate: %.2f\tyaw_control: %.2f\n",ptam_yaw_rate_pilot,-500.0*180.0*(ptam_rpy[2]-ptam_yaw_0)/M_PI);
            i_2=0;
        }

}
//#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

//#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    char wp_status='h';


//        if(contador_1++==10){  //heartbeat
//            contador_1=0;
//            hal.uartA->print("m");
//            //hal.uartE->print("e");
//    }

    switch(ptam_tray_mode){
        case Ptam_tray_Home:
            ptam_pos_target[0]=ptam_posVel_0[0];
            ptam_pos_target[1]=ptam_posVel_0[1];
            ptam_pos_target[2]=ptam_posVel_0[2];
            if(wp_index<WP_NUM){
                ptam_tray_mode=Ptam_tray_Hold;
                hold_count=0;
            }
        break;

        case Ptam_tray_Go:
                ptam_pos_target[0]=ptam_wp[0][wp_index];
                ptam_pos_target[1]=ptam_wp[1][wp_index];
                ptam_pos_target[2]=ptam_wp[2][wp_index];
                wp_index++;
                ptam_tray_mode=Ptam_tray_Hold;
                hold_count=0;
        break;

        case Ptam_tray_Hold:
            if(die_in_target(DIST_TARGET)){
                hold_count++;
            }
            else hold_count=0;
            if(hold_count>=20.0){
                ptam_tray_mode=(wp_index<WP_NUM)?Ptam_tray_Go:Ptam_tray_Home;
            }    //if close to the target for two consecutive seconds, go to the next point
        break;

        default:                                    //just for security
            ptam_pos_target[0]=ptam_posVel_0[0];
            ptam_pos_target[1]=ptam_posVel_0[1];
            ptam_pos_target[2]=ptam_posVel_0[2];
        break;
    }

    if((contador_2++)==10){
            contador_2=0;
            //send a
          const int16_t wp_pck_lenght = 10;
          int16_t pos_send_wp;
          char buffer_wp[wp_pck_lenght];
          float pos_temp_wp=0;
          buffer_wp[0]='P';	//header
          buffer_wp[ wp_pck_lenght -1]=0;
          buffer_wp[1]='w';	//pack type 'w'= waypoints status + target position

        for(int i1=0;i1<3;i1++){
            pos_temp_wp=sat_die(ptam_pos_target[i1]-ptam_posVel_0[i1],-4.0,4.0)*0x7fff/4.0;
            pos_send_wp=(int16_t)pos_temp_wp;
            buffer_wp[2*i1+3]=pos_send_wp&0xff;
            buffer_wp[2*i1+2]=pos_send_wp>>8;
        }//for

            buffer_wp[8]=wp_status;

            buffer_wp[wp_pck_lenght-1]=0;
        for(int i1=0;i1<wp_pck_lenght-1;i1++) {
            buffer_wp[wp_pck_lenght-1]^=buffer_wp[i1];
            hal.uartC->print(buffer_wp[i1]);
        }//for
        hal.uartC->print(buffer_wp[wp_pck_lenght-1]);

    }
}
//#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

int8_t Copter::readPTAM(unsigned char dato){
    static int16_t pack_index, pack_size;

	packPos[pack_index]=dato;
	int8_t packFlag=0;

	if(pack_index==pack_size-1){
		unsigned char exor=0;
		int m,k;

		for(k=0;k<pack_index;k++) exor^=packPos[k];
		if(packPos[pack_index]==exor){
			switch(packPos[1]){
				case 'o':	//ptam_pos_vel
				    for(int16_t i1=0;i1<6;i1++){    //position and velocity
                        ptam_pos_vel[i1]=(float)((int16_t)packPos[2*i1+2]<<8|(uint16_t)packPos[2*i1+3])*(20.0/32767.0);	//scale to maximum +-20m
                        ptam_pos_vel[i1]=ptam_pos_vel[i1]<=20?ptam_pos_vel[i1]:ptam_pos_vel[i1]-40;
				    }
                    for(int16_t i1=0;i1<3;i1++){    //ptam_rpy
                        ptam_rpy[i1]=(float)((int16_t)packPos[2*i1+14]<<8|(uint16_t)packPos[2*i1+15])*(M_PI/32767.0);	//scale to maximum +-20m
                        ptam_rpy[i1]=ptam_rpy[i1]<=M_PI?ptam_rpy[i1]:ptam_rpy[i1]-2*M_PI;
				    }
				    ptam_rpy[2]=ptam_rpy[2]>0?ptam_rpy[2]-M_PI:ptam_rpy[2]+M_PI;
                 break;
				}
				packFlag=1;   //valid package
			}
		else{
			//pack_index=pack_size;
			//scib_xmit('e');
			for(m=1;m<pack_size;m++){
				if(packPos[m]=='P'){
					for(k=0;k<pack_size-m;k++)
						packPos[k]=packPos[m+k];
                        pack_index=pack_size-m;
					break;
                    }//if
                }	//for
            }		//else
		}		//if pack_index

	pack_index=(pack_index+1)%pack_size;

	if(packPos[0]!='P'){
		pack_index=0;
		}
	switch(packPos[1]){
		case 'o':       //ptam information
			pack_size=21;
			break;
		default:
			pack_size=21;
	}
	return packFlag;
}

void Copter::plot_ptam(){
            //hal.uartC->printf("roll: PD=%.2f\t rc= %d !\n pitch: PD=%.2f\t rc=%d\n",ptam_target_roll,channel_roll->get_control_in(),ptam_target_pitch,channel_pitch->get_control_in());
//             hal.uartC->printf("throttle: %d scaled: %.2f control: %.2f pd: %.2f z_e: %.2f kp_alt: %.2f\n",channel_throttle->get_control_in(),pilot_throttle_scaled,control_alt,
//                                            -g.kp_ptam_alt*(ptam_pos_vel[2]-ptam_posVel_0[2])-g.kd_ptam_alt*ptam_pos_vel[5],ptam_pos_vel[2]-ptam_posVel_0[2],(float)g.kp_ptam_alt);

          int16_t pos_send_ptam;
          char buffer_ptam[CMD_LEN];
          float pos_temp_ptam=0;
          buffer_ptam[0]='P';	//header
          buffer_ptam[CMD_LEN-1]=0;
          buffer_ptam[1]='p';	//pack type 'p'=px4 position + vel

//ptam_pos_vel[3]=20.0*sin(2*3.1416*((float)tiempo)/50.0);
//ptam_pos_vel[4]=20.0*sin(2*3.1416*((float)tiempo)/100.0);
//ptam_pos_vel[5]=20.0*sin(2*3.1416*((float)tiempo)/200.0);
//ptam_pos_vel[0]=20.0*sin(2*3.1416*((float)tiempo)/50.0);
//ptam_pos_vel[1]=20.0*sin(2*3.1416*((float)tiempo)/100.0);
//ptam_pos_vel[2]=20.0*sin(2*3.1416*((float)tiempo)/200.0);

//ptam_pos_vel[5]=30.0;

//float debug_ctl_alt=0.5+0.5*sin(2*3.1416*((float)tiempo)/200.0);
//tiempo++;

        for(int i1=0;i1<6;i1++){
            pos_temp_ptam=sat_die(ptam_pos_vel[i1]-ptam_posVel_0[i1],-MAX_DIST,MAX_DIST)*0x7fff/MAX_DIST;
            pos_send_ptam=(int16_t)pos_temp_ptam;
            buffer_ptam[2*i1+3]=pos_send_ptam&0xff;
            buffer_ptam[2*i1+2]=pos_send_ptam>>8;
        }//for
            pos_temp_ptam=control_alt*0x7fff;
            pos_send_ptam=(int16_t)pos_temp_ptam;
            buffer_ptam[15]=pos_send_ptam&0xff;
            buffer_ptam[14]=pos_send_ptam>>8;

            pos_temp_ptam=sat_die(ptam_target_roll,-MAX_ANGLE_PTAM,MAX_ANGLE_PTAM)*0x7fff/MAX_ANGLE_PTAM;
            pos_send_ptam=(int16_t)pos_temp_ptam;
            buffer_ptam[17]=pos_send_ptam&0xff;
            buffer_ptam[16]=pos_send_ptam>>8;

            pos_temp_ptam=sat_die(ptam_target_pitch,-MAX_ANGLE_PTAM,MAX_ANGLE_PTAM)*0x7fff/MAX_ANGLE_PTAM;
            pos_send_ptam=(int16_t)pos_temp_ptam;
            buffer_ptam[19]=pos_send_ptam&0xff;
            buffer_ptam[18]=pos_send_ptam>>8;

            pos_temp_ptam=sat_die(ptam_rpy[2]-ptam_yaw_0,-M_PI,M_PI)*0x7fff/M_PI;
            pos_send_ptam=(int16_t)pos_temp_ptam;
            buffer_ptam[21]=pos_send_ptam&0xff;
            buffer_ptam[20]=pos_send_ptam>>8;

            buffer_ptam[CMD_LEN-1]=0;
        for(int i1=0;i1<CMD_LEN-1;i1++) {
            buffer_ptam[CMD_LEN-1]^=buffer_ptam[i1];
            hal.uartC->print(buffer_ptam[i1]);
            }
            hal.uartC->print(buffer_ptam[CMD_LEN-1]);
            return;
}

bool Copter::die_in_target(float range_tray){
        return die_norm2(ptam_pos_vel[0]-ptam_pos_target[0],ptam_pos_vel[1]-ptam_pos_target[1],ptam_pos_vel[2]-ptam_pos_target[2])<range_tray?true:false;
}

float Copter::die_norm2(float x,float y, float z){
        return sqrt(x*x+y*y+z*z);
}
