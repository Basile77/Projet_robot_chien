#include "ch.h"
#include "hal.h"
#include <chprintf.h>

#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <main.h>

#include <distance_sensor.h>

#define TOF_SLEEP_DURATION_MS	100

// TOF States
#define NO_MEASURE			0
#define DISTANCE_TO_BALL	1

static uint8_t current_TOF_state = DISTANCE_TO_BALL;

//distance measured by TOF
static uint16_t distTOF = 0;

// Semaphore
static BSEMAPHORE_DECL(distance_info_sem, TRUE);

// TOF function prototype
void distance_to_ball_handler(void);

static THD_WORKING_AREA(waDistanceDetec, 256);
static THD_FUNCTION(DistanceDetec, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {

    	// Sets the correct TOF state by checking main state
    	if (get_current_main_state() == FIND_BALL ||
    		get_current_main_state() == GET_BALL ||
			get_current_main_state() == BACK_HOME) {
    		current_TOF_state = DISTANCE_TO_BALL;
    	} else {
    		current_TOF_state = NO_MEASURE;
    	}

    	switch(current_TOF_state) {
    	case NO_MEASURE:
    		chThdSleepMilliseconds(TOF_SLEEP_DURATION_MS);
    		break;
    	case DISTANCE_TO_BALL:
    		distance_to_ball_handler();
    		chThdSleepMilliseconds(TOF_SLEEP_DURATION_MS);
    	}

    }
}

void distance_to_ball_handler(void) {
	distTOF = VL53L0X_get_dist_mm();
	chBSemSignal(&distance_info_sem);
	//chprintf((BaseSequentialStream *)&SD3, "Distance = %d mm \n", distTOF);
}

uint16_t get_distTOF(void) {
	return distTOF;
}

void distanceDetec_start(void) {
	chThdCreateStatic(waDistanceDetec, sizeof(waDistanceDetec), NORMALPRIO, DistanceDetec, NULL);
}

void wait_sem_TOF(void) {
	chBSemWait(&distance_info_sem);
}
