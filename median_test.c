#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>

void swap_f(float *a, float *b) {
    float c = *a;
    *a = *b;
    *b = c;
}

float find_median_select(float *queue, uint8_t size, uint8_t k) {
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
        return find_median_select(queue, pointer2 + 1, k);
    return find_median_select(queue + pointer2 + 1, size - pointer2 - 1, k - pointer2 - 1);
}

float find_median(float *queue, uint8_t size) {
    return find_median_select(queue, size, size/2);
}

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


int main() {
    float array[] = {2, 3, 80, 6, 2, 3};
    uint8_t size = sizeof(array) / sizeof(array[0]); 
    filter_sensor_value(array, size, 1);
    for (int i = 0; i < size; i++)
        printf("%f ", array[i]);
    printf("\n");
    return 0;
}
