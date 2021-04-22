#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <distance_sensor.h>

#define TAILLE_BUFFER 		10
#define DIST_INIT_TOF		500
#define DIST_INIT 			50

//Different possible mode

#define NOT_MOVING			0
#define MOVE_CENTER			1
#define LOOKING_FOR_BALL 	2
#define GO_TO_BALL			3
#define GO_BACK_HOME		4

#define GENERAL_TIME_SLEEP 100

//Current mode of this thread
static int8_t current_mode = NOT_MOVING;



//Static parameters

static int16_t position = 0;
static float distance = DIST_INIT;
static uint16_t dist_TOF = DIST_INIT_TOF;



//handler for different mode

void go_to_ball_handler(uint8_t erreur_cancel_);
void go_back_home_handler(void);



static THD_WORKING_AREA(waDeplacement_robot, 256);
static THD_FUNCTION(Deplacement_robot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

	uint8_t erreur_cancel = 0;

    while(1){
        time = chVTGetSystemTime();


        switch (current_mode){

    	case NOT_MOVING:
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case MOVE_CENTER:
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case LOOKING_FOR_BALL:
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case GO_TO_BALL:
    		go_to_ball_handler(erreur_cancel);
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case GO_BACK_HOME:
    		go_back_home_handler();
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

        }

    }
}



void go_to_ball_handler(uint8_t erreur_cancel_){
	//chprintf((BaseSequentialStream *)&SD3, "Distance moyenne = %d mm \n",  moy_dist_TOF);
	wait_sem();
	chprintf((BaseSequentialStream *)&SD3, "Distance 1= %d mm \n", dist_TOF);

	// Supprimer les 10 premières valeurs
	if (erreur_cancel_ == 10){
		dist_TOF = get_distTOF();
	}
	else {
		++erreur_cancel_;
		dist_TOF = 500;
	}


	distance = get_distance_cm();
	position = get_line_position();
   if (dist_TOF > 50){
		right_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT - (position - IMAGE_BUFFER_SIZE/2));
		left_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT + (position - IMAGE_BUFFER_SIZE/2));
	}

	else if (dist_TOF < 50 ){
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		current_mode = GO_BACK_HOME;
		}
}


void go_back_home_handler(void){
	right_motor_set_speed(-500);
	left_motor_set_speed(-500);
}


void Deplacement_robot_start(void){
	chThdCreateStatic(waDeplacement_robot, sizeof(waDeplacement_robot), NORMALPRIO, Deplacement_robot, NULL);
}
