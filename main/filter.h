#include <stdlib.h>

#ifndef MY_FILTER
#define MY_FILTER

void swap_f(float*, float*); 
float find_stddev(float*, uint16_t);
void filter_sensor_value(float*, uint16_t, uint16_t); 
float find_rank(float*, uint16_t, uint16_t);
#define find_median(array, size) find_rank((array), (size), (size)/2)
#define find_min(array, size) find_rank((array), (size), 0)
#define find_max(array, size) find_rank((array), (size), (size)-1)

#endif // MY_FILTER
