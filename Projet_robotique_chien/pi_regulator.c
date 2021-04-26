#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <leds.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <distance_sensor.h>

#define TAILLE_BUFFER 		10
#define DIST_INIT_TOF		500
#define DIST_INIT 			50
#define DIST_TO_CENTER		20
#define SPEED_ROTATE		200
#define SPEED_FORWARD		600
#define CORRECTION 			0.05
#define WHEEL_PERIMETER		13.0f // [cm]
#define NSTEP_ONE_TURN		1000 // number of step for 1 turn of the motor
#define TIME_CONST			(10/SPEED_FORWARD*NSTEP_ONE_TURN/WHEEL_PERIMETER)
//defini 2 fois ATTENTION
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)


//Different possible mode

#define NOT_MOVING			0
#define MOVE_CENTER			1
#define LOOKING_FOR_BALL 	2
#define GO_TO_BALL			3
#define GO_BACK_CENTER		4
#define GO_BACK_HOME		5

#define GENERAL_TIME_SLEEP 100

//Current mode of this thread
static int8_t current_mode = LOOKING_FOR_BALL;



//Static parameters

static int16_t position = 0;
static float distance = DIST_INIT;
static uint16_t dist_TOF = DIST_INIT_TOF;
static uint16_t dist_to_memorise = DIST_INIT_TOF;
static uint32_t move_center_counter = 0;
static uint16_t angle_counter = 0;
static uint16_t angle_counter_half_turn = TIME_CONST*PERIMETER_EPUCK/4;
static uint16_t speed_counter = 0;

//handler for different mode

void look_for_ball_handler(void);
void go_to_ball_handler(void);
void go_back_center_handler(void);
void go_back_home_handler(void);
void move_center_handler(void);


static THD_WORKING_AREA(waDeplacement_robot, 256);
static THD_FUNCTION(Deplacement_robot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t erreur_cancel = 0;

    while(1){
    	chprintf((BaseSequentialStream *)&SD3, "TEST= %d mm \n", dist_TOF);

        switch (current_mode){

    	case NOT_MOVING:
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case MOVE_CENTER:
    		move_center_handler();
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case LOOKING_FOR_BALL:
    		set_led(LED1, 1);
    		look_for_ball_handler();
    		break;

    	case GO_TO_BALL:

    		// Supprimer les 10 premières valeurs
    		if (erreur_cancel > 10){
        	set_led(LED3, 1);
        	go_to_ball_handler();
    		}
    		else {
    			++erreur_cancel;
    		}

    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case GO_BACK_CENTER:
    		set_led(LED5, 1);
    		go_back_center_handler();
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case GO_BACK_HOME:
    		set_led(LED7, 1);
    		go_back_home_handler();
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

        }

    }
}
void move_center_handler(){

	right_motor_set_speed(-SPEED_FORWARD);
	left_motor_set_speed(-SPEED_FORWARD);

	if (move_center_counter < (DIST_TO_CENTER*TIME_CONST)){
		right_motor_set_speed(-SPEED_FORWARD);
		left_motor_set_speed(-SPEED_FORWARD);
		move_center_counter++;
	}
	else {current_mode = LOOKING_FOR_BALL;}
}


void look_for_ball_handler(){


	right_motor_set_speed(SPEED_ROTATE);
	left_motor_set_speed(-SPEED_ROTATE);
	distance = get_distance_cm();
	position = get_line_position();
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	position = get_line_position();
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	position = get_line_position();
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	position = get_line_position();
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	position = get_line_position();
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	position = get_line_position();
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	while ((position < IMAGE_BUFFER_SIZE/2*(1 - CORRECTION))|| (position > IMAGE_BUFFER_SIZE/2*(1 + CORRECTION)) || (distance == 50)){
		++angle_counter;
		distance = get_distance_cm();
		position = get_line_position();
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}

	current_mode = GO_TO_BALL;
	dist_to_memorise = get_distTOF();



//	position = get_line_position();
//	distance = get_distance_cm();
//	if (position > IMAGE_BUFFER_SIZE/2*(1 - CORRECTION) && position < IMAGE_BUFFER_SIZE/2*(1 + CORRECTION) && distance < 50){
//		current_mode = GO_TO_BALL;
//		dist_to_memorise = get_distTOF();
//	}
//	else {
//		++angle_counter;
//		right_motor_set_speed(SPEED_ROTATE);
//		left_motor_set_speed(-SPEED_ROTATE);
//	}
}



void go_to_ball_handler(){
	chprintf((BaseSequentialStream *)&SD3, "Distance moyenne = %d mm \n",  dist_TOF);
	wait_sem();
	chprintf((BaseSequentialStream *)&SD3, "Distance 1= %d mm \n", dist_TOF);
	dist_TOF = get_distTOF();


	chprintf((BaseSequentialStream *)&SD3, "DistanceTOF = %d mm \n", dist_TOF);
	distance = get_distance_cm();
	position = get_line_position();
   if (dist_TOF > 50){
		//right_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT - (position - IMAGE_BUFFER_SIZE/2));
		//left_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT + (position - IMAGE_BUFFER_SIZE/2));
		right_motor_set_speed(SPEED_FORWARD);
		left_motor_set_speed(SPEED_FORWARD );
	}

	else if (dist_TOF < 50 ){
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		current_mode = GO_BACK_CENTER;
		}
}


void go_back_center_handler(void){

	if (angle_counter_half_turn > 0){
		right_motor_set_speed(SPEED_ROTATE);
		left_motor_set_speed(-SPEED_ROTATE);
		--angle_counter_half_turn;
	}

	else if (speed_counter < dist_to_memorise*TIME_CONST){
		right_motor_set_speed(SPEED_FORWARD);
		left_motor_set_speed(SPEED_FORWARD);
		speed_counter++;
	}

	if (speed_counter > (dist_to_memorise*TIME_CONST)){
		current_mode = GO_BACK_HOME;
	}
}

void go_back_home_handler(void){

	if (angle_counter + TIME_CONST*PERIMETER_EPUCK/4 > 0){
		right_motor_set_speed(SPEED_ROTATE);
		left_motor_set_speed(-SPEED_ROTATE);
		--angle_counter;
	}

	if (angle_counter + TIME_CONST*PERIMETER_EPUCK/4 < 0){
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}
}

void Deplacement_robot_start(void){
	chThdCreateStatic(waDeplacement_robot, sizeof(waDeplacement_robot), NORMALPRIO, Deplacement_robot, NULL);
}
