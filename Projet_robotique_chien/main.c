//Modified File from TP4

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
#include <deplacement_robot.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <msgbus/messagebus.h>
#include <leds.h>
#include "audio/play_melody.h"
#include <process_image.h>
#include "distance_sensor.h"
#include "audio_processing.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


 static uint8_t current_main_state = WAIT_FOR_COLOR;


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
	//start the time-of-flight sensor
	VL53L0X_start();

	spi_comm_start();

	//stars the threads for deplacement_robot and the processing of the image

	deplacement_robot_start();

	process_image_start();

	// Start the melody thread
	dac_start();
	playMelodyStart();

	// Start the thread to sense distance (TOF)
	distanceDetec_start();

    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
     mic_start(&processAudioData);


    /* Infinite loop. */
    while (1) {

    	//Decide on what to do depending on the current state
    	switch(current_main_state) {
    	case WAIT_FOR_COLOR:
    		wait_sem_audio();
    		current_main_state = RETURN_CENTER;
    		break;

    	case RETURN_CENTER:
    		wait_sem_motor();
    		current_main_state = FIND_BALL;
    		break;

    	case FIND_BALL:
    		wait_sem_motor();
    		current_main_state = GET_BALL;
    		break;

    	case GET_BALL:
    		wait_sem_motor();
    		current_main_state = BACK_HOME;
    		break;

    	case BACK_HOME:
    		wait_sem_motor();
    		current_main_state = WAIT_FOR_COLOR;
    		playNote(NOTE_C4, 100);
    		playNote(NOTE_E4, 100);
    		playNote(NOTE_G4, 100);
    		playNote(NOTE_C5, 100);
    		playNote(NOTE_E5, 100);
    		playNote(NOTE_G5, 150);
    		for (uint8_t i = 0; i < 4; ++i){
    			set_rgb_led(LED2, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    			set_rgb_led(LED4, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    			set_rgb_led(LED6, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    			set_rgb_led(LED8, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    			set_rgb_led(LED2, 0, 0, 0);
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    			set_rgb_led(LED4, 0, 0, 0);
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    			set_rgb_led(LED6, 0, 0, 0);
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    			set_rgb_led(LED8, 0, 0, 0);
        		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		}
    		break;
    	}


    	chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    }
}


uint8_t get_current_main_state(void){
	return current_main_state;
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
