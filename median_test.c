#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

void swap_f(float *a, float *b) {
    float c = *a;
    *a = *b;
    *b = c;
}

float find_rank(float *queue, uint8_t size, uint8_t k) {
    if (size == 1) return queue[0];
    // Place randomly selected pivot to the beginning of array
    uint16_t pointer1 = 1;
    uint16_t pointer2 = size-1;

    // Partition
    while (pointer1 <= pointer2) {
        if (queue[pointer1] < queue[0]) {
            pointer1++;
        } else {
            swap_f(queue + pointer1, queue + pointer2);
            pointer2--;
        }
    }
    // Put the pivot in the right place
    swap_f(queue, queue + pointer2);
    // Recurse
    if (pointer2 >= k)
        return find_rank(queue, pointer2 + 1, k);
    return find_rank(queue + pointer2 + 1, size - pointer2 - 1, k - pointer2 - 1);
}

#define find_median(array, size) find_rank((array), (size), (size)/2)
#define find_min(array, size) find_rank((array), (size), 0)
#define find_max(array, size) find_rank((array), (size), (size)-1)

void filter_sensor_value(float *array, uint16_t size, uint8_t window_size) {
    float filtered_array[size];
    for (int i = 0; i < size; i++) {
        float window[2*window_size + 1];
        window[window_size] = array[i];
        for (int j = 0; j < window_size; j++) {
            window[j] = array[(i - 1 - j + size) % size];
            window[window_size + j + 1] = array[(i + j + 1) % size];
        }
        filtered_array[i] = find_median(window, 2*window_size + 1);
    }
    memcpy(array, filtered_array, sizeof(float)*size);
}

float standard_deviation(float *array, uint16_t size) {
    float mean = 0;
    float result = 0;
    for (uint16_t i = 0; i < size; i++)
        mean += array[i];
    mean /= (float)size;
    for (uint16_t i = 0; i < size; i++)
        result += powf((array[i] - mean), 2); 
    result /= (float)size;
    result = sqrt(result);
    return result;
}

int main() {
    float array[] = {2, 3, 80, 6, 2, 3};
    uint8_t size = sizeof(array) / sizeof(array[0]); 
    filter_sensor_value(array, size, 1);
    for (int i = 0; i < size; i++)
        printf("%f ", array[i]);
    printf("\n");
    float array2[] = {85, 86, 100, 76, 81, 93, 84, 99, 71, 69, 93, 85, 81, 87, 89};
    uint8_t size2 = sizeof(array2) / sizeof(array2[0]); 
    float stddev = standard_deviation(array2, size2);
    printf("Standard deviation: %f\n", stddev);
    return 0;
}
