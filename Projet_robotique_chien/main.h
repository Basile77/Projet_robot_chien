#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				15
#define MIN_LINE_WIDTH			80
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			3.0f
#define MAX_DISTANCE 			25.0f

#define WAIT_FOR_COLOR		0 		// Transition to next : whistle
#define RETURN_CENTER		1		// Transition to next : destination reached
#define FIND_BALL			2		// Transition to next : ball found
#define GET_BALL			3		// Transition to next : ball reached
#define BACK_HOME			4		// Transition to next : destination reached

#define GENERAL_TIME_SLEEP 100

#define NO_COLOR				0
#define BLUE					1
#define GREEN					2
#define RED						3



/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

uint8_t get_current_main_state(void);

#ifdef __cplusplus
}
#endif

#endif
