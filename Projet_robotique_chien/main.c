#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <sensors/proximity.h>

#include <sensors/VL53L0X/VL53L0X.h>


#include <pi_regulator.h>
#include <process_image.h>
#include "distance_sensor.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    //proximity_start();

    // Init the communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//starts the proximity sensors
	proximity_start();
	calibrate_ir();
	//start the time-of-flight sensor
	VL53L0X_start();


	//stars the threads for the pi regulator and the processing of the image
	//Deplacement_robot_start();
	//process_image_start();


	// Start the thread to sense proximity
	proximityDetec_start();

	// Start the thread to sense distance
	distanceDetec_start();


    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    //mic_start(&processAudioData);

	uint8_t actual_color = NO_COLOR;


    /* Infinite loop. */
    while (1) {
    	//waits 1 second

    	chThdSleepMilliseconds(500);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
