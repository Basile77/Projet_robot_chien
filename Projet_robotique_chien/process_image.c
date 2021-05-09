#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>
#include <audio_processing.h>

#include <process_image.h>

#define MEMORIZE_COLOR		0
#define FIND_COLOR			1

#define NOT_CAPTURING		0
#define CAPTURING			1

#define COLOR_FULL_SCALE 	256
#define COLOR_MARGIN 		1.1
#define COLOR_THRESHOLD		(uint16_t)COLOR_FULL_SCALE*0.4
#define MINIMUM_COLOR		180

static float distance_cm = 10;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint8_t color_memory = NO_COLOR;

static bool process_image_current_state = MEMORIZE_COLOR;
static bool capture_image_current_state = CAPTURING;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */

uint16_t extract_line_width(uint8_t *main_color, uint8_t *color2, uint8_t *color3){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += main_color[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(main_color[i] < mean && main_color[i+WIDTH_SLOPE] > mean &&
		    		main_color[i+WIDTH_SLOPE] > MINIMUM_COLOR)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;

		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(main_color[i] < mean && main_color[i-WIDTH_SLOPE] > mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
				line_position = 0;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
			line_position = 0;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;

			line_position = 0;
			line_not_found = 1;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found || main_color[(begin + end)/2] < color2[(begin + end)/2]
		|| main_color[(begin + end)/2] < color3[(begin + end)/2]){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
}


int mean_buff(uint8_t *buffer) {
	int mean = 0;
	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;
	return mean;
}

uint8_t extract_color(uint8_t *buffer_green, uint8_t *buffer_red, uint8_t *buffer_blue){

	if ((buffer_green[IMAGE_BUFFER_SIZE/2] > COLOR_THRESHOLD) &&
			(buffer_red[IMAGE_BUFFER_SIZE/2] < COLOR_THRESHOLD) &&
			(buffer_blue[IMAGE_BUFFER_SIZE/2] < COLOR_THRESHOLD)) {
		set_rgb_led(LED6, 0, RGB_MAX_INTENSITY/2, 0);
//		chprintf((BaseSequentialStream *)&SD3, "GREEN, ");
		return GREEN;
	} else if ((buffer_green[IMAGE_BUFFER_SIZE/2] < COLOR_THRESHOLD) &&
			(buffer_red[IMAGE_BUFFER_SIZE/2] > COLOR_THRESHOLD) &&
			(buffer_blue[IMAGE_BUFFER_SIZE/2] < COLOR_THRESHOLD)) {
		set_rgb_led(LED6, RGB_MAX_INTENSITY/2, 0, 0);
//		chprintf((BaseSequentialStream *)&SD3, "RED, ");
		return RED;
	} else if ((buffer_green[IMAGE_BUFFER_SIZE/2] < COLOR_THRESHOLD) &&
			(buffer_red[IMAGE_BUFFER_SIZE/2] < COLOR_THRESHOLD) &&
			(buffer_blue[IMAGE_BUFFER_SIZE/2] > COLOR_THRESHOLD)) {
		set_rgb_led(LED6, 0, 0, RGB_MAX_INTENSITY/2);
//		chprintf((BaseSequentialStream *)&SD3, "BLUE, ");
		return BLUE;
	}
//	 chprintf((BaseSequentialStream *)&SD3, "NO COLOR, GREEN = %d, RED = %d, BLUE = %d",
//			buffer_green[IMAGE_BUFFER_SIZE/2], buffer_red[IMAGE_BUFFER_SIZE/2],
//			buffer_blue([IMAGE_BUFFER_SIZE/2]));

	return NO_COLOR;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 200, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	po8030_set_awb(0);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){

    	// Sets the correct camera state by checking main state
		if (get_current_main_state() == RETURN_CENTER ||
			get_current_main_state() == BACK_HOME) {
			capture_image_current_state = NOT_CAPTURING;
		} else {
			capture_image_current_state = CAPTURING;
		}

    	switch(capture_image_current_state) {
    	case CAPTURING :
            //starts a capture
    		dcmi_capture_start();
    		//waits for the capture to be done
    		wait_image_ready();
    		//signals an image has been captured
    		chBSemSignal(&image_ready_sem);
    		break;
    	case NOT_CAPTURING :
    		//Do nothing until state has changed
    		chThdSleepMilliseconds(GENERAL_TIME_SLEEP);
    		break;
    	}

    }
}

static THD_WORKING_AREA(waProcessImage, 4096);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;



	uint8_t *img_buff_ptr;

	static uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t image_blue[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};

	static uint16_t lineWidth = 0;

	static bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts red, blue and green colors
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){

			image_red[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8);
			image_blue[i/2] = (((uint8_t)img_buff_ptr[i+1]&0x1F)<<3);
			image_green[i/2] = ((((uint8_t)img_buff_ptr[i]&0x07)<<5) + (((uint8_t)img_buff_ptr[i+1]&0xE0)>>3));

		}
		if (get_current_main_state() != WAIT_FOR_COLOR) {
			process_image_current_state = FIND_COLOR;
		} else {
			process_image_current_state = MEMORIZE_COLOR;
		}

		switch(process_image_current_state) {
		case MEMORIZE_COLOR:
			if (extract_color(image_green, image_red, image_blue) != NO_COLOR) {
				color_memory = extract_color(image_green, image_red, image_blue);
			}
			break;
		case FIND_COLOR:

			//search for a line in the image and gets its width in pixels
			switch(color_memory){
			case RED:
				for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
					if (image_red[i] < (uint8_t)COLOR_FULL_SCALE/COLOR_MARGIN) {
						image_red[i] *= COLOR_MARGIN;
					}
				}
				lineWidth = extract_line_width(image_red, image_green, image_blue);
				break;
			case GREEN:
				for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
					if (image_red[i] < (uint8_t)COLOR_FULL_SCALE/COLOR_MARGIN) {
						image_red[i] *= COLOR_MARGIN*2;
					}
				}
				lineWidth = extract_line_width(image_green, image_red, image_blue);
				break;
			case BLUE:
				lineWidth = extract_line_width(image_blue, image_green, image_red);
				break;
			}

			//converts the width into a distance between the robot and the camera
			if(lineWidth){
				distance_cm = PXTOCM/lineWidth;
			}
			else{distance_cm = 50;}
			if(send_to_computer){
				//sends to the computer the image
				SendUint8ToComputer(image_green, IMAGE_BUFFER_SIZE);
			}
			//invert the bool
			send_to_computer = !send_to_computer;

			break;
		}
    }
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

uint8_t get_color(void){
	return color_memory;
}





void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
