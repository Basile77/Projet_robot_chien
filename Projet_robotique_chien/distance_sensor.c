#include "ch.h"
#include "hal.h"
#include <chprintf.h>

#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>

#include <distance_sensor.h>

#define PROXIMITY_THRESHOLD		300
#define PROX_SLEEP_DURATION_MS	100
#define TOF_SLEEP_DURATION_MS	100

#define NO_MEASURE			0

// Proximity States
#define ARRIVAL				1
static uint8_t current_prox_state = NO_MEASURE;

// TOF States
#define DISTANCE_TO_BALL	1
static uint8_t current_TOF_state = NO_MEASURE;

static uint16_t distTOF = 0;

// Semaphore
static BSEMAPHORE_DECL(wall_contact_sem, TRUE);
static BSEMAPHORE_DECL(distance_info_sem, TRUE);



// Proximity / TOF functions
void arrival_handler(void);
void distance_to_ball_handler(void);

static THD_WORKING_AREA(waProximityDetec, 256);
static THD_FUNCTION(ProximityDetec, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t contact_counter = 0;

    while(1) {
    	switch(current_prox_state) {
    	case NO_MEASURE:
    		// does nothing, no handler needed
    		chThdSleepMilliseconds(PROX_SLEEP_DURATION_MS);
    		break;

    	case ARRIVAL:
    		arrival_handler();
    		chThdSleepMilliseconds(PROX_SLEEP_DURATION_MS);
    		break;
    	}
    }
}

void arrival_handler(void) {
	// A COMPLETER
}

static THD_WORKING_AREA(waDistanceDetec, 256);
static THD_FUNCTION(DistanceDetec, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {
    	switch(current_TOF_state) {
    	case NO_MEASURE:
    		chThdSleepMilliseconds(TOF_SLEEP_DURATION_MS);
    		break;


    	}


    	distTOF = VL53L0X_get_dist_mm();
    	chprintf((BaseSequentialStream *)&SD3, "Distance = %d mm \n", distTOF);
		chBSemSignal(&distance_info_sem);
        chThdSleepMilliseconds(500);
    }
}



uint16_t get_distTOF(void) {
	return distTOF;
}

void proximityDetec_start(void) {
	chThdCreateStatic(waProximityDetec, sizeof(waProximityDetec), NORMALPRIO, ProximityDetec, NULL);
}

void distanceDetec_start(void) {
	chThdCreateStatic(waDistanceDetec, sizeof(waDistanceDetec), NORMALPRIO, DistanceDetec, NULL);
}

void wait_sem(void) {
	chBSemWait(&distance_info_sem);
}
