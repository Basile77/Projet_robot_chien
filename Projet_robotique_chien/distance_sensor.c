#include "ch.h"
#include "hal.h"
#include <chprintf.h>

#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <distance_sensor.h>

static uint16_t distTOF = 0;

static THD_WORKING_AREA(waProximityDetec, 256);
static THD_FUNCTION(ProximityDetec, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {
    	int value = get_prox(0);
    	chprintf((BaseSequentialStream *)&SD3, "front right = %d \n", value);
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
}

void distanceDetec_start(void) {
	chThdCreateStatic(waDistanceDetec, sizeof(waDistanceDetec), NORMALPRIO, DistanceDetec, NULL);
}
