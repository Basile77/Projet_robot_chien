#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H


//Start the TOF thread
void distanceDetec_start(void);

//Return the dist given by the TOF
uint16_t get_distTOF(void);
void wait_sem_TOF(void);

#endif
