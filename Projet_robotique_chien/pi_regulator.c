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
#define CORRECTION 			0.4
#define CORRECTION_BIS 		0.2
#define WHEEL_PERIMETER		13.0f // [cm]
#define NSTEP_ONE_TURN		1000 // number of step for 1 turn of the motor
#define TIME_CONST			(1000.0f/GENERAL_TIME_SLEEP/SPEED_FORWARD*NSTEP_ONE_TURN/WHEEL_PERIMETER)
#define TIME_CONST_SLOW			(1000.0f/GENERAL_TIME_SLEEP/(SPEED_FORWARD*0.2)*NSTEP_ONE_TURN/WHEEL_PERIMETER)

//defini 2 fois ATTENTION
#define PI                  3.1415f
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
#define LOOK_BALL_TIME_SLEEP 5

//Current mode of this thread
static int8_t current_mode = NOT_MOVING;



//Static parameters

static int16_t position = 0;
static float distance = DIST_INIT;
static uint16_t dist_TOF = DIST_INIT_TOF;
static float dist_to_memorise = DIST_INIT_TOF;
static uint16_t dist_to_memorise_2 = 20.0f;
static uint32_t move_center_counter = 0;

static uint32_t angle_counter = 0;
static uint32_t angle_counter_2 = 0;
static uint16_t angle_counter_alignement = 0;
static uint16_t go_back_center_counter = 0;


static uint32_t move_counter = 0;


//handler for different mode

void look_for_ball_handler(void);
void go_to_ball_handler(void);
void go_back_center_handler(void);
void go_back_home_handler(void);
void move_center_handler(void);

uint8_t move(uint8_t nb_step , uint16_t counter);
uint8_t actual_mode(uint8_t main_state);

static BSEMAPHORE_DECL(sendMotoState_sem, TRUE);

static THD_WORKING_AREA(waDeplacement_robot, 512);
static THD_FUNCTION(Deplacement_robot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t erreur_cancel = 0;
	right_motor_set_speed(0);
	left_motor_set_speed(0);

    while(1){

        switch (current_mode){

    	case NOT_MOVING:

    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    		current_mode = actual_mode(get_current_main_state());
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case MOVE_CENTER:

    		move_center_handler();
    		chBSemSignal(&sendMotoState_sem);
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);

    		current_mode = actual_mode(get_current_main_state());
    		break;

    	case LOOKING_FOR_BALL:

    		look_for_ball_handler();
    		chBSemSignal(&sendMotoState_sem);
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);

    		current_mode = actual_mode(get_current_main_state());
    		break;

    	case GO_TO_BALL:

    		// Supprimer les 10 premi�res valeurs
    		if (erreur_cancel > 10){
        	go_to_ball_handler();
    		chBSemSignal(&sendMotoState_sem);
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);

    		current_mode = actual_mode(get_current_main_state());
    		}
    		else {
    			++erreur_cancel;
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		}
    		break;

    	case GO_BACK_CENTER:
     		set_led(LED5, 0);
    		go_back_center_handler();
    		set_led(LED1, 0);
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case GO_BACK_HOME:

    		set_led(LED3, 0);
    		go_back_home_handler();
     		set_led(LED5, 0);
    		chBSemSignal(&sendMotoState_sem);
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);


    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
		current_mode = actual_mode(get_current_main_state());

    		break;

        }

    }
}
void move_center_handler(){



	right_motor_set_speed(-SPEED_FORWARD);
	left_motor_set_speed(-SPEED_FORWARD);
	uint8_t destination_reached = 0 ;
	move_center_counter = 0;


	while (!destination_reached){
		uint8_t nb_step = (int)DIST_TO_CENTER*TIME_CONST;
		destination_reached = move(nb_step, move_center_counter);
		++move_center_counter;
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}

}


void look_for_ball_handler(){

	angle_counter = 0;
	angle_counter_2 = 0;

	right_motor_set_speed(SPEED_FORWARD*0.2);
	left_motor_set_speed(-SPEED_FORWARD*0.2);
	set_led(LED3, 1);
	distance = get_distance_cm();
	position = get_line_position();
	angle_counter += SPEED_FORWARD*0.2;
	++angle_counter_2;
	chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);
	angle_counter += SPEED_FORWARD*0.2;
	++angle_counter_2;
	position = get_line_position();
	chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);
	angle_counter += SPEED_FORWARD*0.2;
	++angle_counter_2;
	position = get_line_position();
	chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);
	angle_counter += SPEED_FORWARD*0.2;
	++angle_counter_2;
	position = get_line_position();
	chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);
	position = get_line_position();
	angle_counter += SPEED_FORWARD*0.2;
	++angle_counter_2;
	chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);
	position = get_line_position();
	distance = get_distance_cm();
	angle_counter += SPEED_FORWARD*0.2;
	++angle_counter_2;
	chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);

	chprintf((BaseSequentialStream *)&SD3, "angle_counter = %d mm \n\n\n\n\n\n\n\n", angle_counter);
	while ((position < IMAGE_BUFFER_SIZE/2*(1 - CORRECTION))|| (position > IMAGE_BUFFER_SIZE/2*(1 + CORRECTION)) || (distance == 50)){
	//while (position == 0){

		angle_counter += SPEED_FORWARD*0.2;
		++angle_counter_2;
		distance = get_distance_cm();
		position = get_line_position();
		chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);

	}

	chprintf((BaseSequentialStream *)&SD3, "angle_counter = %d mm \n\n\n\n\n\n\n\n", angle_counter);
	right_motor_set_speed(0);
	left_motor_set_speed(0);

	chThdSleepMilliseconds(GENERAL_TIME_SLEEP*10);
	set_led(LED3, 0);

	position = get_line_position();
	distance = get_distance_cm();

//	uint8_t destination_reached = 0 ;
//
//	if (position < IMAGE_BUFFER_SIZE/2){
//		right_motor_set_speed(SPEED_FORWARD*0.1);
//		left_motor_set_speed(-SPEED_FORWARD*0.1);
//	}
//	else {
//		right_motor_set_speed(-SPEED_FORWARD*0.1);
//		left_motor_set_speed(SPEED_FORWARD*0.1);
//	}
//
//	while (!destination_reached){
//
//		++angle_counter;
//		distance = get_distance_cm();
//		position = get_line_position();
//		if ((position > IMAGE_BUFFER_SIZE/2*(1 - CORRECTION_BIS)) && (position < IMAGE_BUFFER_SIZE/2*(1 + CORRECTION_BIS)) && (distance < 50)){
//			chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP*0.5);
//			destination_reached = 1;
//		}
//	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);

//	uint16_t taille_ligne = PXTOCM/distance;
//	uint8_t destination_reached = 0 ;
//	angle_counter_alignement = 0;
//	float alpha = asin((IMAGE_BUFFER_SIZE/2-position)/taille_ligne);
//	if (alpha > 0){
//		right_motor_set_speed(SPEED_FORWARD*0.2);
//		left_motor_set_speed(-SPEED_FORWARD*0.2);
//	}
//	else {
//		right_motor_set_speed(-SPEED_FORWARD*0.2);
//		left_motor_set_speed(SPEED_FORWARD*0.2);
//		alpha = -alpha;
//	}
//	uint8_t nb_step = (uint8_t)(PERIMETER_EPUCK*alpha/(2*PI)*TIME_CONST_SLOW);
//	while (!destination_reached){
//		destination_reached = move(nb_step, angle_counter_alignement);
//		++angle_counter_alignement;
//		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
//	}




	dist_to_memorise = (float)get_distTOF()/10. - 2.5;
	chprintf((BaseSequentialStream *)&SD3, "A MEMORISER = %f CM \n", dist_to_memorise);

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

	wait_sem_TOF();
	dist_TOF = get_distTOF();
	uint16_t old_position = position;
	while (dist_TOF > 25){

		wait_sem_TOF();
		dist_TOF = get_distTOF();
		distance = get_distance_cm();
		position = get_line_position();
		if (position == 0){
			position = old_position;
		}
		if (dist_TOF > 100){
			right_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT);
			left_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT);
//			right_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT - (position - IMAGE_BUFFER_SIZE/2));
//			left_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT + (position - IMAGE_BUFFER_SIZE/2));
		}
		else {
			right_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT);
			left_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT);
		}
//		right_motor_set_speed(SPEED_FORWARD);
//		left_motor_set_speed(SPEED_FORWARD);
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP*0.5);

	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);




//chprintf((BaseSequentialStream *)&SD3, "Distance moyenne = %d mm \n",  dist_TOF);
//wait_sem_TOF();
//chprintf((BaseSequentialStream *)&SD3, "Distance 1= %d mm \n", dist_TOF);
//dist_TOF = get_distTOF();
//
//
//chprintf((BaseSequentialStream *)&SD3, "DistanceTOF = %d mm \n", dist_TOF);
//distance = get_distance_cm();
//position = get_line_position();
//f (dist_TOF > 50){
//	right_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT - (position - IMAGE_BUFFER_SIZE/2));
//	left_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT + (position - IMAGE_BUFFER_SIZE/2));
//	//right_motor_set_speed(SPEED_FORWARD);
//	//left_motor_set_speed(SPEED_FORWARD );
//}
//
//else if (dist_TOF < 50 ){
//	right_motor_set_speed(0);
//	left_motor_set_speed(0);
//	current_mode = GO_BACK_CENTER;
//	}
}


void go_back_center_handler(void){


	move_counter = 0;
	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(-SPEED_FORWARD);
	++move_counter;
	uint8_t destination_reached = 0 ;


	while (!destination_reached){
		uint8_t nb_step = (uint8_t)(PERIMETER_EPUCK/2*TIME_CONST);
		destination_reached = move(nb_step, move_counter);
		++move_counter;
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}

//	while (!destination_reached){
//		destination_reached = move(DIST_TO_CENTER, move_center_counter);
//		++move_center_counter;
//		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
//	}


	go_back_center_counter = 0;
	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(SPEED_FORWARD);
	destination_reached = 0 ;
	while (!destination_reached){
		uint8_t nb_step = (int)(dist_to_memorise*TIME_CONST);
		destination_reached = move(nb_step, go_back_center_counter);
		++go_back_center_counter;
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}

	right_motor_set_speed(0);
	left_motor_set_speed(0);
	current_mode = GO_BACK_HOME;
}


void go_back_home_handler(void){


	right_motor_set_speed(SPEED_FORWARD*0.2);
	left_motor_set_speed(-SPEED_FORWARD*0.2);
	uint8_t destination_reached = 0 ;
	uint32_t nb_step = 	angle_counter_2;
	chprintf((BaseSequentialStream *)&SD3, "angle_counter = %d mm \n\n\n\n\n\n\n\n", angle_counter_2);
	angle_counter_2 = 0;
	while (!destination_reached){
		destination_reached = move(nb_step, angle_counter_2);
		++angle_counter_2;
		chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);
	}

	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(SPEED_FORWARD);
	move_center_counter = 0;

	destination_reached = 0;
	move_center_counter = 0;
	while (!destination_reached){
		uint8_t nb_step = (int)(DIST_TO_CENTER*TIME_CONST);
		destination_reached = move(nb_step, move_center_counter);
		++move_center_counter;
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}


	current_mode = NOT_MOVING;
}


uint8_t move(uint8_t nb_step , uint16_t counter){


	if(counter < nb_step){
		return 0;
	}
	return 1;
}

//uint8_t move(float distance, uint16_t counter){
//
//	float nb_step = distance*TIME_CONST;
//	if(counter < (int)nb_step){
//		chprintf((BaseSequentialStream *)&SD3, "counter = %d mm \n", counter);
//		chprintf((BaseSequentialStream *)&SD3, "nb step total = %d mm \n", (int)nb_step);
//		return 0;
//	}
//	return 1;
//}

uint8_t actual_mode(uint8_t main_state){
	switch(main_state) {
	case WAIT_FOR_COLOR:
		return NOT_MOVING;
		break;
	case RETURN_CENTER:
		return MOVE_CENTER;
		break;
	case FIND_BALL:
		return LOOKING_FOR_BALL;
		break;
	case GET_BALL:
		return GO_TO_BALL;
		break;
	case BACK_HOME:
		return GO_BACK_CENTER;
		break;
	}
	return NOT_MOVING;
}

void Deplacement_robot_start(void){
	chThdCreateStatic(waDeplacement_robot, sizeof(waDeplacement_robot), NORMALPRIO, Deplacement_robot, NULL);
}

void wait_sem_motor(void) {
	chBSemWait(&sendMotoState_sem);
}

