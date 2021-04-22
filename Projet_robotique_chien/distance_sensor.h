#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

void proximityDetec_start(void);
void distanceDetec_start(void);

uint16_t get_distTOF(void);


void wait_sem(void);

#endif
