#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <process_image.h>


static THD_WORKING_AREA(waDeplacement_robot, 256);
static THD_FUNCTION(Deplacement_robot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t position = 0;
    float distance = 50.;
    int16_t mode = MODE_0;

    while(1){
        time = chVTGetSystemTime();

        if (mode == MODE_0){

			distance = get_distance_cm();
			dist_TOF = get_distTOF();
			position = get_line_position();
			//computes the speed to give to the motors
			//distance_cm is modified by the image processing thread
		   if (dist_TOF > 5){
				right_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT - (position - IMAGE_BUFFER_SIZE/2));
				left_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT + (position - IMAGE_BUFFER_SIZE/2));
			}
			else if (dist_TOF < 5 && ){
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				mode = MODE_1;

			}
        }

        if (mode == MODE_1){
        	right_motor_set_speed(500);
        	left_motor_set_speed(-500);
        	mode = MODE_2;
        }

        if (mode == MODE_2){
        	right_motor_set_speed(-500);
        	left_motor_set_speed(-500);

        }
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void Deplacement_robot_start(void){
	chThdCreateStatic(waDeplacement_robot, sizeof(waDeplacement_robot), NORMALPRIO, Deplacement_robot, NULL);
}
