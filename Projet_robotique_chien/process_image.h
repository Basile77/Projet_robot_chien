#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H




void process_image_start(void);

//Return the color in memory
uint8_t get_color(void);

//Return the position of the center of the line
uint16_t get_line_position(void);

//Return the distance to the object calculated from the width of the line
float get_distance_cm(void);
#endif /* PROCESS_IMAGE_H */
