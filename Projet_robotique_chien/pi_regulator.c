#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <distance_sensor.h>

#define TAILLE_BUFFER 		10
#define DIST_MAX 			500


static THD_WORKING_AREA(waDeplacement_robot, 256);
static THD_FUNCTION(Deplacement_robot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t position = 0;
    float distance = 50.;
    int16_t mode = MODE_0;
    uint16_t dist_TOF = DIST_MAX;

	uint16_t historique_dist_TOF[TAILLE_BUFFER] = {500};
	uint8_t position_buffer = 0;

	uint16_t somme = 0;
    uint16_t moy_dist_TOF = DIST_MAX;


    while(1){
        time = chVTGetSystemTime();


        switch (mode){


			case MODE_0:

				distance = get_distance_cm();
				dist_TOF = get_distTOF();
				historique_dist_TOF[position_buffer] = dist_TOF;
				position = get_line_position();
			   if (moy_dist_TOF > 50){
					right_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT - (position - IMAGE_BUFFER_SIZE/2));
					left_motor_set_speed(distance/30*MOTOR_SPEED_LIMIT + (position - IMAGE_BUFFER_SIZE/2));
				}
				else if (moy_dist_TOF < 50 ){
					right_motor_set_speed(0);
					left_motor_set_speed(0);
					mode = MODE_1;

					}


				++position_buffer;
				if (position_buffer == TAILLE_BUFFER){position_buffer = 0;}

				somme = 0;
				for (uint8_t i = 0; i<TAILLE_BUFFER; ++i){
					somme += historique_dist_TOF[i];
				}
				moy_dist_TOF = somme/(TAILLE_BUFFER);

		    	chprintf((BaseSequentialStream *)&SD3, "Distance moyenne = %d mm \n",  moy_dist_TOF);
		    	chprintf((BaseSequentialStream *)&SD3, "Distance 1= %d mm \n", historique_dist_TOF[1]);

				break;



			case MODE_1:
				right_motor_set_speed(500);
				left_motor_set_speed(-500);
				mode = MODE_2;

				break;


			case MODE_2:
				right_motor_set_speed(-500);
				left_motor_set_speed(-500);

				break;

        }
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}

void Deplacement_robot_start(void){
	chThdCreateStatic(waDeplacement_robot, sizeof(waDeplacement_robot), NORMALPRIO, Deplacement_robot, NULL);
}
