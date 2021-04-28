#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#define MEMORIZE_COLOR		0
#define FIND_COLOR			1

#define CAPTURING			0
#define NOT_CAPTURING		1

static float distance_cm = 10;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint8_t color_memory = NO_COLOR;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */

uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean && buffer[i+WIDTH_SLOPE] > 130)
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
		        if(buffer[i] < mean && buffer[i-WIDTH_SLOPE] > mean)
		        {
		            end = i;
		            stop = 1;
		     //for(uint8_t l = begin; l<end; ++l){
		     //	if (buffer[l] < mean){
		     //       end = 0;
		     //	}
		     //}


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
			//wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
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


//uint16_t extract_line_width(uint8_t *buffer){
//
//	uint32_t mean = 0;
//	uint16_t start = 0;
//	uint16_t i = 0;
//	uint16_t end = 0;
//	uint16_t width = 0;
//
//	for(uint16_t k = 0 ; k < IMAGE_BUFFER_SIZE ; k++){
//		mean += buffer[k];
//	}
//	mean /= IMAGE_BUFFER_SIZE;
//
//	while(start == 0 && i < IMAGE_BUFFER_SIZE){
//		if(buffer[i] < mean && buffer[i+5] > mean*1.8 && buffer[i+5] > 150){
//			start = i;
//		}
//		++i;
//	}
//
//	uint16_t j = start;
//	if (j != 0){
//		while(end == 0 && j < IMAGE_BUFFER_SIZE){
//			if(buffer[j] > 1.8*mean && buffer[j+5] < mean && buffer[j+5] < 80){
//				end = j;
//			}
//			++j;
//		}
//	}
//
//	width = end - start;
//	if (width < 30){
//		width = 0;
//	}
//
//	line_position = (end + start)/2;
//	return width;
//
//}



uint8_t extract_color(uint8_t *buffer_vert, uint8_t *buffer_rouge, uint8_t *buffer_bleu){

	uint16_t taille_vert = extract_line_width(buffer_vert);
	uint16_t taille_bleu = extract_line_width(buffer_bleu);
	uint16_t taille_rouge = extract_line_width(buffer_rouge);

	if (taille_vert > 0 && (buffer_vert[IMAGE_BUFFER_SIZE/2] > buffer_rouge[IMAGE_BUFFER_SIZE/2])
						&& (buffer_vert[IMAGE_BUFFER_SIZE/2] > buffer_bleu[IMAGE_BUFFER_SIZE/2])){
		return VERT;
	}

	if (taille_bleu > 0 && (buffer_bleu[IMAGE_BUFFER_SIZE/2] > buffer_rouge[IMAGE_BUFFER_SIZE/2])
						&& (buffer_bleu[IMAGE_BUFFER_SIZE/2] > buffer_vert[IMAGE_BUFFER_SIZE/2])){
		return BLEU;
	}

	if (taille_rouge > 0 && (buffer_rouge[IMAGE_BUFFER_SIZE/2] > buffer_vert[IMAGE_BUFFER_SIZE/2])
						&& (buffer_rouge[IMAGE_BUFFER_SIZE/2] > buffer_bleu[IMAGE_BUFFER_SIZE/2])){
		return ROUGE;
	}

	return NO_COLOR;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 50, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;

	uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_blue[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};

	uint16_t lineWidth = 0;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image_red[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8);
			image_blue[i/2] = (((uint8_t)img_buff_ptr[i+1]&0x1F)<<3);
			image_green[i/2] = ((((uint8_t)img_buff_ptr[i]&0x07)<<5) + (((uint8_t)img_buff_ptr[i+1]&0xE0)>>3));

		}


		//color_memory = extract_color(image_green, image_red, image_blue);


		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image_green);


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
    }
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

uint8_t get_couleur(void){
	return color_memory;
}





void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
