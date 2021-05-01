#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <msgbus/messagebus.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>

// Microphone states
#define NO_MEASURE 			0
#define WAIT_FOR_WHISTLE 	1

#define NO_MEASURE 			0
#define WAIT_FOR_WHISTLE 	1

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);
static BSEMAPHORE_DECL(sendAudioState_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

static bool current_mic_state = WAIT_FOR_WHISTLE;

#define MIN_VALUE_THRESHOLD	10000

#define MIN_FREQ		64	//1000Hz we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		128	//2000Hz we don't analyze after this index to not use resources for nothing

#define FREQ_WHISTLE	96 // 96 = 1500Hz
#define FREQ_WHISTLE_L	FREQ_WHISTLE-5 //FREQ_WHISTLE - 78Hz
#define FREQ_WHISTLE_H	FREQ_WHISTLE+5 //FREQ_WHISTLE + 78Hz

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//go forward
	if(max_norm_index >= FREQ_WHISTLE_L && max_norm_index <= FREQ_WHISTLE_H){
		chBSemSignal(&sendAudioState_sem);
		chprintf((BaseSequentialStream *)&SD3, "Audio signal sent");
		current_mic_state = NO_MEASURE;
	} else {
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}

}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	static uint16_t nb_samples = 0;

	switch(current_mic_state) {
	case NO_MEASURE:
		if (get_current_state() == WAIT_FOR_COLOR){
			current_mic_state = WAIT_FOR_WHISTLE;
		}
		break;
	case WAIT_FOR_WHISTLE:
		/*
		*
		*	We get 160 samples per mic every 10ms
		*	So we fill the samples buffers to reach
		*	1024 samples, then we compute the FFTs.
		*
		*/

		//loop to fill the buffers
		for(uint16_t i = 0 ; i < num_samples ; i+=4){
			//construct an array of complex numbers. Put 0 to the imaginary part
			micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
			micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
			micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
			micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

			nb_samples++;

			micRight_cmplx_input[nb_samples] = 0;
			micLeft_cmplx_input[nb_samples] = 0;
			micBack_cmplx_input[nb_samples] = 0;
			micFront_cmplx_input[nb_samples] = 0;

			nb_samples++;

			//stop when buffer is full
			if(nb_samples >= (2 * FFT_SIZE)){
				break;
			}
		}

		if(nb_samples >= (2 * FFT_SIZE)){
			/*	FFT proccessing
			*
			*	This FFT function stores the results in the input buffer given.
			*	This is an "In Place" function.
			*/

			doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
			doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
			doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

			/*	Magnitude processing
			*
			*	Computes the magnitude of the complex numbers and
			*	stores them in a buffer of FFT_SIZE because it only contains
			*	real numbers.
			*
			*/
			arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
			arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
			arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

			nb_samples = 0;

			sound_remote(micLeft_output);

		}
		break;
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}

void wait_sem_audio(void) {
	chprintf((BaseSequentialStream *)&SD3, "J'attends actuellement l'audio");
	chBSemWait(&sendAudioState_sem);
	chprintf((BaseSequentialStream *)&SD3, "J'ai re�u");
}
