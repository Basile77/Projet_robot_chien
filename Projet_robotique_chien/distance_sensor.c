#include "ch.h"
#include "hal.h"
#include <chprintf.h>

#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>

#include <distance_sensor.h>

#define PROXIMITY_THRESHOLD		300

#define NO_MEASURE			0

// Proximity States
#define ARRIVAL				1

// TOF States
#define DISTANCE_TO_BALL	1

static uint16_t distTOF = 0;

// Semaphore
static BSEMAPHORE_DECL(wall_contact_sem, TRUE);

static THD_WORKING_AREA(waProximityDetec, 256);
static THD_FUNCTION(ProximityDetec, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t contact_counter = 0;

    while(1) {
    	int prox_value = get_prox(0);
    	chprintf((BaseSequentialStream *)&SD3, "front right = %d \n", prox_value);
    	if (prox_value > PROXIMITY_THRESHOLD) {
    		contact_counter = 5;
    	}
    	if (contact_counter > 0) {
    		left_motor_set_speed(-1000);
    		right_motor_set_speed(-1000);
    		contact_counter -= 1;
    	}
    	if (contact_counter == 1) {
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    		contact_counter = 0;
    	}
    	chprintf((BaseSequentialStream *)&SD3, "contact_counter = %d \n", contact_counter);
        chThdSleepMilliseconds(100);
    }
}

static THD_WORKING_AREA(waDistanceDetec, 256);
static THD_FUNCTION(DistanceDetec, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {
    	distTOF = VL53L0X_get_dist_mm();
    	chprintf((BaseSequentialStream *)&SD3, "Distance = %d mm \n", distTOF);
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
