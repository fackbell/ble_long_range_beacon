#ifndef GPS_H__
#define GPS_H__

#include "common.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define NUMBER_MESSAGE_WORD 16
#define MESSAGE_BUFFER_SIZE 12

extern int32_t latitude, longitude;
extern char lat_direction, long_direction;
extern int16_t altitude;

uint32_t gps_config(void);
void gps_event_init(void);
void uart_init(nrf_uart_baudrate_t baud);
uint8_t gps_comm(char gps_buffer[NUMBER_MESSAGE_WORD][MESSAGE_BUFFER_SIZE], const char *message_seach_pattern);

#endif
