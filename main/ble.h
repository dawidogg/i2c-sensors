// Borda Academy 2024 
// Embedded Systems Developer Intern & New Graduate Candidate Assignment
// Author: Denis Davidoglu
#ifndef MY_BLE
#define MY_BLE

typedef struct {
    char name[16];
    float stddev;
    float max;
    float min;
    float median;
} ble_packet_t;

void ble_transmit_packet(ble_packet_t*, uint8_t);

#endif // MY_BLE
