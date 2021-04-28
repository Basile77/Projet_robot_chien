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
#define DIST_INIT_TOF		500.0f
#define DIST_INIT 			50
#define DIST_TO_CENTER		20.0f
#define SPEED_ROTATE		200
#define SPEED_FORWARD		600
#define CORRECTION 			0.8
#define WHEEL_PERIMETER		13.0f // [cm]
#define NSTEP_ONE_TURN		1000 // number of step for 1 turn of the motor
#define TIME_CONST			(10.0f/SPEED_FORWARD*NSTEP_ONE_TURN/WHEEL_PERIMETER)

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
static int8_t current_mode = MOVE_CENTER;



//Static parameters

static int16_t position = 0;
static float distance = DIST_INIT;
static uint16_t dist_TOF = DIST_INIT_TOF;
static float dist_to_memorise = DIST_INIT_TOF;
static uint16_t dist_to_memorise_2 = 20.0f;
static uint32_t move_center_counter = 0;
static uint16_t angle_counter = 0;
static uint16_t angle_counter_half_turn = 0;
static uint16_t go_back_center_counter = 0;

//handler for different mode

void look_for_ball_handler(void);
void go_to_ball_handler(void);
void go_back_center_handler(void);
void go_back_home_handler(void);
void move_center_handler(void);

int move(float distance, uint8_t counter);

static BSEMAPHORE_DECL(sendMotoState_sem, TRUE);

static THD_WORKING_AREA(waDeplacement_robot, 256);
static THD_FUNCTION(Deplacement_robot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t erreur_cancel = 0;

    while(1){
    	chprintf((BaseSequentialStream *)&SD3, "TEST= %d mm \n", dist_TOF);

        switch (current_mode){

    	case NOT_MOVING:

    		right_motor_set_speed(0);
    		left_motor_set_speed(0);

    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case MOVE_CENTER:

    		move_center_handler();
    		break;

    	case LOOKING_FOR_BALL:
    		set_led(LED1, 1);
    		look_for_ball_handler();

    		chBSemSignal(&sendMotoState_sem);
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;

    	case GO_TO_BALL:

    		// Supprimer les 10 premières valeurs
    		if (erreur_cancel > 10){
        	go_to_ball_handler();
    		}
    		else {
    			++erreur_cancel;
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		}
    		chBSemSignal(&sendMotoState_sem);
    		break;

    	case GO_BACK_CENTER:

    		go_back_center_handler();
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		chBSemSignal(&sendMotoState_sem);
    		break;

    	case GO_BACK_HOME:
    		set_led(LED7, 1);
    		go_back_home_handler();
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		chBSemSignal(&sendMotoState_sem);
    		break;

        }

    }
}
void move_center_handler(){


	right_motor_set_speed(-SPEED_FORWARD);
	left_motor_set_speed(-SPEED_FORWARD);
	uint8_t destination_reached = 0 ;



	while (!destination_reached){
		destination_reached = move(DIST_TO_CENTER, move_center_counter);
		++move_center_counter;
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}

	current_mode = LOOKING_FOR_BALL;

}


void look_for_ball_handler(){

	set_led(LED5, 1);
	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(-SPEED_FORWARD);
	distance = get_distance_cm();
	position = get_line_position();
	++angle_counter;
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	++angle_counter;
	position = get_line_position();
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	++angle_counter;
	position = get_line_position();
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	++angle_counter;
	position = get_line_position();
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	position = get_line_position();
	++angle_counter;
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	position = get_line_position();
	distance = get_distance_cm();
	++angle_counter;
	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);

	set_led(LED5, 0);
	//while ((position < IMAGE_BUFFER_SIZE/2*(1 - CORRECTION))|| (position > IMAGE_BUFFER_SIZE/2*(1 + CORRECTION)) || (distance == 50)){
	while (position == 0){

		++angle_counter;
		distance = get_distance_cm();
		position = get_line_position();
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
		set_led(LED5, 1);
	}

	current_mode = GO_TO_BALL;
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	dist_to_memorise = (float)get_distTOF()/10. - 5;
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

	while (dist_TOF > 50){
    	set_led(LED3, 1);
		wait_sem_TOF();
		dist_TOF = get_distTOF();
		distance = get_distance_cm();
		position = get_line_position();
		chprintf((BaseSequentialStream *)&SD3, "Distance = %f mm \n", distance);
		chprintf((BaseSequentialStream *)&SD3, "Position = %d mm \n", position);
		right_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT - (position - IMAGE_BUFFER_SIZE/2));
		left_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT + (position - IMAGE_BUFFER_SIZE/2));
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP*0.5);
		set_led(LED7, 1);
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	current_mode = GO_BACK_CENTER;




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

	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(-SPEED_FORWARD);
	++angle_counter_half_turn;
	uint8_t destination_reached = 0 ;
	while (!destination_reached){
		destination_reached = move(PERIMETER_EPUCK/2, angle_counter_half_turn);
		++angle_counter_half_turn;
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}
	set_led(LED7, 1);
	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(SPEED_FORWARD);
	destination_reached = 0 ;

	while (!destination_reached){
		destination_reached = move(dist_to_memorise, go_back_center_counter);
		++go_back_center_counter;
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}

	current_mode = GO_BACK_HOME;
}

void go_back_home_handler(void){
	set_led(LED7, 0);

	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(-SPEED_FORWARD);
	uint8_t destination_reached = 0 ;
	float distace_to_face_exit = angle_counter/TIME_CONST;
	angle_counter = 0;
	++angle_counter;
	++angle_counter;
	while (!destination_reached){
		chprintf((BaseSequentialStream *)&SD3, "counter_exit_angle = %d mm \n", angle_counter);
		destination_reached = move(distace_to_face_exit, angle_counter);
		++angle_counter;
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}

	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(SPEED_FORWARD);

	destination_reached = 0;
	move_center_counter = 0;
	while (!destination_reached){
		destination_reached = move(DIST_TO_CENTER, move_center_counter);
		++move_center_counter;
		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
	}

	set_led(LED1, 0);
	set_led(LED3, 0);
	set_led(LED5, 0);
	set_led(LED7, 0);

	current_mode = NOT_MOVING;
}


int move(float distance, uint8_t counter){

	float nb_step = distance*TIME_CONST;
	if(counter < (int)nb_step){
		chprintf((BaseSequentialStream *)&SD3, "counter = %d mm \n", counter);
		chprintf((BaseSequentialStream *)&SD3, "nb step total = %d mm \n", (int)nb_step);
		return 0;
	}
	return 1;
}



void Deplacement_robot_start(void){
	chThdCreateStatic(waDeplacement_robot, sizeof(waDeplacement_robot), NORMALPRIO, Deplacement_robot, NULL);
}

void wait_sem_motor(void) {
	chBSemWait(&sendMotoState_sem);
}

