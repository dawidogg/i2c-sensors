// Borda Academy 2024 
// Embedded Systems Developer Intern & New Graduate Candidate Assignment
// Author: Denis Davidoglu
#include <stdio.h>
#include "ble.h"

void ble_transmit_packet(ble_packet_t *packet, uint8_t size) {
    printf("| %16s | %16s | %16s | %16s | %16s |\n", "Name", "Stddev", "Max", "Min", "Median");
    printf("+------------------+------------------+------------------+------------------+------------------+\n");
    for (uint8_t i = 0; i < size; i++) {
        printf("| %16s | %16f | %16f | %16f | %16f |\n", packet[i].name, packet[i].stddev, packet[i].max, packet[i].min, packet[i].median);
    }
    printf("\n");
}
