//Modified File from TP4



#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <main.h>
#include <camera/po8030.h>
#include <leds.h>
#include <audio_processing.h>
#include <process_image.h>


#define MEMORIZE_COLOR			0
#define FIND_COLOR				1

#define NOT_CAPTURING			0
#define CAPTURING				1

#define COLOR_FULL_SCALE 		255
#define COLOR_MARGIN_TEST 		1.1						//How much the main color must be higher than the others
#define COLOR_MARGIN_BOOST 		1.3						//of how much the main color is boosted 		(Both are empirical)
#define COLOR_THRESHOLD			(uint16_t)COLOR_FULL_SCALE*0.7

#define MINIMUM_COLOR			180						//Minimum color intensity to be recognised		(empirical)

static uint16_t distance_mm = 100;							//Distance to the line
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//position of the center of the line
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

	// Assurance that the correct color line was found
	if(line_not_found || main_color[(begin + end)/2] < color2[(begin + end)/2]*COLOR_MARGIN_TEST
		|| main_color[(begin + end)/2] < color3[(begin + end)/2]*COLOR_MARGIN_TEST){
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

// Simple mean function
int mean_buff(uint8_t *buffer) {
	int mean = 0;
	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;
	return mean;
}

// Determines what color is being shown and turns on the corresponding RBG LED
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

	return NO_COLOR;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 201, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

	//Disable Auto exposure and auto white balance
	po8030_set_awb(0);
	po8030_set_ae(0);
	// Set a different gain for each color (empirical)
	po8030_set_rgb_gain(0x64, 0x64, 0x75);


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

		// Check that the correct process state is set
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
				for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
					if (image_red[i] < (uint8_t)COLOR_FULL_SCALE/COLOR_MARGIN_BOOST) {
						image_red[i] *= COLOR_MARGIN_BOOST;
					}
					else {image_red[i] = COLOR_FULL_SCALE;}
				}
				lineWidth = extract_line_width(image_red, image_green, image_blue);
				break;
			case GREEN:
				for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
					if (image_green[i] < (uint8_t)COLOR_FULL_SCALE/COLOR_MARGIN_BOOST) {
						image_green[i] *= COLOR_MARGIN_BOOST;
					}
					else {image_green[i] = COLOR_FULL_SCALE;}
				}
				lineWidth = extract_line_width(image_green, image_red, image_blue);
				break;
			case BLUE:
				for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
					if (image_blue[i] < (uint8_t)COLOR_FULL_SCALE/COLOR_MARGIN_BOOST) {
						image_blue[i] *= COLOR_MARGIN_BOOST;
					}
					else {image_blue[i] = COLOR_FULL_SCALE;}
				}
				lineWidth = extract_line_width(image_blue, image_green, image_red);
				break;
			}

			//converts the width into a distance between the robot and the camera
			if(lineWidth){
				distance_mm = (uint16_t)PXTOCM/lineWidth;
			}
			else{distance_mm = 500;}


			break;
		}
    }
}

float get_distance_mm(void){
	return distance_mm;
}

uint16_t get_line_position(void){
	return line_position;
}

uint8_t get_color(void){
	return color_memory;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+1, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO+1, CaptureImage, NULL);
}
