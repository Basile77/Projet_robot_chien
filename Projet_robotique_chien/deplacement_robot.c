//Original File

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <deplacement_robot.h>
#include <leds.h>
#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <distance_sensor.h>
#include "audio/play_melody.h"


#define TAILLE_BUFFER 		10
#define DIST_INIT_TOF		500
#define DIST_INIT 			50
#define DIST_TO_CENTER		20
#define DIST_MIN_TO_OBJECT	25
#define SPEED_FORWARD		600
#define SPEED_ROTATE		SPEED_FORWARD*0.2
#define CORRECTION 			0.25					//set a range for the detection of the object in look_ball_hander()
#define WHEEL_PERIMETER		13.0f // [cm]
#define NSTEP_ONE_TURN		1000 // number of step for 1 turn of the motor
#define TIME_CONST			(1000.0f/GENERAL_MOTOR_TIME_SLEEP/SPEED_FORWARD*NSTEP_ONE_TURN/WHEEL_PERIMETER)
#define TIME_CONST_SLOW		(1000.0f/LOOK_BALL_TIME_SLEEP/(SPEED_ROTATE)*NSTEP_ONE_TURN/WHEEL_PERIMETER)


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

#define GENERAL_MOTOR_TIME_SLEEP 	100
#define LOOK_BALL_TIME_SLEEP		5

//Current mode of this thread
static int8_t current_motor_state = NOT_MOVING;



//Static parameters

static uint16_t dist_to_memorise = DIST_INIT_TOF;	//distance of the object when detected
static uint16_t angle_counter = 0;				//counter to remember the angle from looking to home to the object
static uint16_t move_counter = 0;				//counter used in every move function


//handler for different mode

void move_center_handler(void);					//function called when moving center
void look_for_ball_handler(void);				//function called when looking around for the ball
void go_to_ball_handler(void);					//function called when going to the ball
void go_back_center_handler(void);				//function called when ball found, semi-circle and go center
void go_back_home_handler(void);				//function called when robot in center, turn around and go back home


//different function to move (explanation in definition)

void go_to(uint16_t nb_step , uint16_t counter);
void go_to_slow(uint16_t nb_step , uint16_t counter);
bool dest_reached(uint16_t nb_step , uint16_t counter);


//set current state by looking at main state
uint8_t current_mode(uint8_t main_state);




static BSEMAPHORE_DECL(sendMotoState_sem, TRUE);

static THD_WORKING_AREA(waDeplacement_robot, 512);
static THD_FUNCTION(Deplacement_robot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	right_motor_set_speed(0);
	left_motor_set_speed(0);

    while(1){

        switch (current_motor_state){

    	case NOT_MOVING:
    		current_motor_state = current_mode(get_current_main_state());
    		chThdSleepMilliseconds(GENERAL_MOTOR_TIME_SLEEP);
    		break;

    	case MOVE_CENTER:

    		move_center_handler();
    		chBSemSignal(&sendMotoState_sem);
    		chThdSleepMilliseconds(GENERAL_MOTOR_TIME_SLEEP);


    		current_motor_state = current_mode(get_current_main_state());
    		break;

    	case LOOKING_FOR_BALL:

    		look_for_ball_handler();
    		chBSemSignal(&sendMotoState_sem);
    		chThdSleepMilliseconds(GENERAL_MOTOR_TIME_SLEEP);

    		current_motor_state = current_mode(get_current_main_state());
    		break;

    	case GO_TO_BALL:

        	go_to_ball_handler();
    		chBSemSignal(&sendMotoState_sem);
    		chThdSleepMilliseconds(GENERAL_MOTOR_TIME_SLEEP);
    		current_motor_state = current_mode(get_current_main_state());
    		break;

    	case GO_BACK_CENTER:
    		go_back_center_handler();
    		chThdSleepMilliseconds(GENERAL_MOTOR_TIME_SLEEP);
    		break;

    	case GO_BACK_HOME:


    		go_back_home_handler();
    		chBSemSignal(&sendMotoState_sem);
    		chThdSleepMilliseconds(GENERAL_MOTOR_TIME_SLEEP);
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    		current_motor_state = current_mode(get_current_main_state());

    		break;

        }

    }
}


void move_center_handler(){

	right_motor_set_speed(-SPEED_FORWARD);
	left_motor_set_speed(-SPEED_FORWARD);
	move_counter = 0;
	uint16_t nb_step = DIST_TO_CENTER*TIME_CONST;
	go_to(nb_step, move_counter);


}


void look_for_ball_handler(){

	angle_counter = 0;
	right_motor_set_speed(SPEED_ROTATE);
	left_motor_set_speed(-SPEED_ROTATE);
	uint16_t position = get_line_position();
	uint16_t dist_TOF = get_distTOF();

	do{
		++angle_counter;
		dist_TOF = get_distTOF();
		position = get_line_position();
		chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);
	}while ((position < IMAGE_BUFFER_SIZE/2*(1 - CORRECTION)) || (position > IMAGE_BUFFER_SIZE/2*(1 + CORRECTION)) || (dist_TOF > 250));
	//leave the loop if the position of the center of the line is near the middle (corrected by the CORRECTION factor) and if distance is greater than 500, this last one is to not be disturbed by noise

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	chThdSleepMilliseconds(GENERAL_MOTOR_TIME_SLEEP);

	position = get_line_position();
	dist_to_memorise = get_distTOF() - DIST_MIN_TO_OBJECT;


}



void go_to_ball_handler(){

	wait_sem_TOF();
	uint16_t dist_TOF = get_distTOF();
	uint16_t distance = get_distance_mm();
	int16_t position = get_line_position();
	uint16_t old_position = position;
	while (dist_TOF > DIST_MIN_TO_OBJECT){

		wait_sem_TOF();
		dist_TOF = get_distTOF();
		distance = get_distance_mm();
		position = get_line_position();
		if (position == 0){
			position = old_position;
		}
		right_motor_set_speed(distance*MOTOR_SPEED_LIMIT/30);
		left_motor_set_speed(distance*MOTOR_SPEED_LIMIT/30);
		chThdSleepMilliseconds(GENERAL_MOTOR_TIME_SLEEP);

	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);


}


void go_back_center_handler(void){


	move_counter = 0;
	right_motor_set_speed(SPEED_ROTATE);
	left_motor_set_speed(-SPEED_ROTATE);
	uint16_t nb_step = (PERIMETER_EPUCK/2*TIME_CONST_SLOW);
	go_to_slow(nb_step, move_counter);


	move_counter = 0;
	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(SPEED_FORWARD);
	nb_step = (dist_to_memorise*TIME_CONST/10);
	go_to(nb_step, move_counter);

	right_motor_set_speed(0);
	left_motor_set_speed(0);
	current_motor_state = GO_BACK_HOME;
}


void go_back_home_handler(void){


	uint16_t nb_step = 	angle_counter;
	move_counter = (PERIMETER_EPUCK/2*TIME_CONST_SLOW);




	nb_step =  nb_step % (2*move_counter);

	if (move_counter > nb_step){
		nb_step = move_counter - nb_step;
		right_motor_set_speed(SPEED_ROTATE);
		left_motor_set_speed(-SPEED_ROTATE);
	}
	else {
		nb_step = nb_step - move_counter;
		right_motor_set_speed(-SPEED_ROTATE);
		left_motor_set_speed(SPEED_ROTATE);
	}
	angle_counter = 0;

	go_to_slow(nb_step, angle_counter);

	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(SPEED_FORWARD);
	move_counter = 0;
	nb_step = (DIST_TO_CENTER*TIME_CONST);

	go_to(nb_step, move_counter);
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	current_motor_state = NOT_MOVING;
}


//Return 1 if destination reached, Return 0 oterwise

bool dest_reached(uint16_t nb_step , uint16_t counter){
	if(counter < nb_step){
		return 0;
	}
	return 1;
}



//Function used to move the robot at SPEED_FORWARD for @nb_step times GENERAL_MOTOR_TIME_SLEEP

void go_to(uint16_t nb_step , uint16_t counter){

	bool destination_reached = 0 ;

	while (!destination_reached){
		destination_reached = dest_reached(nb_step, counter);
		++counter;
		chThdSleepMilliseconds(GENERAL_MOTOR_TIME_SLEEP);
	}

}

//Function used to move the robot at SPEED_ROTATE for @nb_step times LOOK_BALL_TIME_SLEEP

void go_to_slow(uint16_t nb_step , uint16_t counter){

	bool destination_reached = 0 ;

	while (!destination_reached){
		destination_reached = dest_reached(nb_step, counter);
		++counter;
		chThdSleepMilliseconds(LOOK_BALL_TIME_SLEEP);
	}

}



uint8_t current_mode(uint8_t main_state){
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

void deplacement_robot_start(void){
	chThdCreateStatic(waDeplacement_robot, sizeof(waDeplacement_robot), NORMALPRIO, Deplacement_robot, NULL);
}

void wait_sem_motor(void) {
	chBSemWait(&sendMotoState_sem);
}

