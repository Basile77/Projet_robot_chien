#include "ch.h"
#include "hal.h"
#include <chprintf.h>

#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>

#include <distance_sensor.h>

#define PROXIMITY_THRESHOLD		300

static uint16_t distTOF = 0;

// Semaphore
static BSEMAPHORE_DECL(wall_contact_sem, TRUE);

static THD_WORKING_AREA(waProximityDetec, 256);
static THD_FUNCTION(ProximityDetec, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {
    	int prox_value = get_prox(0);
    	chprintf((BaseSequentialStream *)&SD3, "front right = %d \n", prox_value);
    	if (prox_value > PROXIMITY_THRESHOLD) {
    		chBSemSignal(&wall_contact_sem);
    	}
        chThdSleepMilliseconds(100);
    }
}

static THD_WORKING_AREA(waWallContact, 256);
static THD_FUNCTION(WallContact, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {
    	left_motor_set_speed(0);
    	right_motor_set_speed(0);
    	chBSemWait(&wall_contact_sem);
    	chprintf((BaseSequentialStream *)&SD3, "Wall Touched !\n");
    	left_motor_set_speed(-1000);
    	right_motor_set_speed(-1000);
    	chThdSleepMilliseconds(500);
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
	chThdCreateStatic(waWallContact, sizeof(waWallContact), NORMALPRIO+1, WallContact, NULL);
}

void distanceDetec_start(void) {
	chThdCreateStatic(waDistanceDetec, sizeof(waDistanceDetec), NORMALPRIO, DistanceDetec, NULL);
}
