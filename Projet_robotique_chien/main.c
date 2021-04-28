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
#include <audio/microphone.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <msgbus/messagebus.h>
#include <leds.h>

#include <pi_regulator.h>
#include <process_image.h>
#include "distance_sensor.h"
#include "audio_processing.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


 static uint8_t current_main_state = WAIT_FOR_COLOR;



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

	Deplacement_robot_start();

	//process_image_start();


	// Start the thread to sense proximity

	//proximityDetec_start();

	// Start the thread to sense distance
	distanceDetec_start();

    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);

//	uint8_t actual_color = NO_COLOR;

	set_led(LED1, 0);

	uint8_t actual_color = NO_COLOR;


    /* Infinite loop. */
    while (1) {

    	switch(current_main_state) {
    	case WAIT_FOR_COLOR:
    		chprintf((BaseSequentialStream *)&SD3, "Current main State = WAIT_FOR_COLOR, ");
    		wait_sem_audio();
    		current_main_state = RETURN_CENTER;
    		set_led(LED1, 1);
    		chThdSleepMilliseconds(500);
    		break;
    	case RETURN_CENTER:
    		chprintf((BaseSequentialStream *)&SD3, "Current main State = RETURN_CENTER, ");
    		wait_sem_motor();
    		current_main_state = FIND_BALL;
    		set_led(LED3, 1);
    		chThdSleepMilliseconds(500);
    		break;
    	case FIND_BALL:
    		chprintf((BaseSequentialStream *)&SD3, "Current main State = FIND_BALL, ");
    		wait_sem_motor();
    		current_main_state = GET_BALL;
    		set_led(LED5, 1);
    		chThdSleepMilliseconds(500);
    		break;
    	case GET_BALL:
    		chprintf((BaseSequentialStream *)&SD3, "Current main State = GET_BALL, ");
    		wait_sem_motor();
    		current_main_state = BACK_HOME;
    		set_led(LED7, 1);
    		chThdSleepMilliseconds(500);
    		break;
    	case BACK_HOME:
    		chprintf((BaseSequentialStream *)&SD3, "Current main State = BACK_HOME");
     		set_led(LED7, 0);
    		wait_sem_motor();
    		current_main_state = WAIT_FOR_COLOR;
    		set_led(LED1, 0);
    		set_led(LED3, 0);
    		set_led(LED5, 0);
    		set_led(LED7, 0);
    		chThdSleepMilliseconds(500);
    		break;
    	}


    	chThdSleepMilliseconds(100);
    }
}

uint8_t get_current_state(void){
	return current_main_state;
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
