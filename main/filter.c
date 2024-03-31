// Borda Academy 2024 
// Embedded Systems Developer Intern & New Graduate Candidate Assignment
// Author: Denis Davidoglu
#include "esp_random.h"
#include "xtensa/hal.h"
#include <math.h>

#include "filter.h"

void swap_f(float *a, float *b) {
    float c = *a;
    *a = *b;
    *b = c;
}

float find_rank(float *array, uint16_t size, uint16_t k) {
    if (size == 0) return 0;
    if (size == 1) return array[0];
    // Place randomly selected pivot to the beginning of array
    swap_f(array, array + (esp_random()%size));
    uint16_t pointer1 = 1;
    uint16_t pointer2 = size-1;

    // Partition
    while (pointer1 <= pointer2) {
        if (array[pointer1] < array[0]) {
            pointer1++;
        } else {
            swap_f(array + pointer1, array + pointer2);
            pointer2--;
        }
    }
    // Put the pivot in the right place
    swap_f(array, array + pointer2);
    // Recurse
    if (pointer2 >= k)
        return find_rank(array, pointer2 + 1, k);
    return find_rank(array + pointer2 + 1, size - pointer2 - 1, k - pointer2 - 1);
}

void filter_sensor_value(float *array, uint16_t size, uint16_t window_size) {
    float filtered_array[size];
    for (int i = 0; i < size; i++) {
        float window[2*window_size + 1];
        window[window_size] = array[i];
        for (int j = 0; j < window_size; j++) {
            window[j] = array[(i - 1 - j + size) % size];
            window[window_size + j + 1] = array[(i + j) % size];
        }
        filtered_array[i] = find_median(window, 2*window_size + 1);
    }
    xthal_memcpy(array, filtered_array, sizeof(float)*size);
}

float find_stddev(float *array, uint16_t size) {
    float result = 0;
    if (size != 0) {
        float mean = 0;
        for (uint16_t i = 0; i < size; i++)
            mean += array[i];
        mean /= (float)size;
        for (uint16_t i = 0; i < size; i++)
            result += powf((array[i] - mean), 2); 
        result /= (float)size;
        result = sqrt(result);
    }
    return result;
}
