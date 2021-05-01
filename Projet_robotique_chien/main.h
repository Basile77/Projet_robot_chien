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
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			3.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#define WAIT_FOR_COLOR		0 		// Transition to next : whistle
#define RETURN_CENTER		1		// Transition to next : destination reached
#define FIND_BALL			2		// Transition to next : ball found
#define GET_BALL			3		// Transition to next : ball reached
#define BACK_HOME			4		// Transition to next : destination reached



#define NO_COLOR				0
#define BLEU					1
#define VERT					2
#define ROUGE					3


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);
uint8_t get_current_state(void);

#ifdef __cplusplus
}
#endif

#endif
