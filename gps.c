#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "gps.h"

#define GPS_CYCLE_ADDRESS 0x10001088                            /**< Address of the cycle time values for GPS. */
#define GPS_TIMER MSEC_TO_UNITS(10000, UNIT_10_MS)	// This is in terms of 100uS. Both set are to 1 sec and multiplied by the user value.

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#define UART_TX_BUF_SIZE 64                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

int32_t latitude, longitude;
char lat_direction, long_direction;
int16_t altitude;

typedef struct
{
    const uint8_t *data;
    uint8_t size;
} message;

static const nrf_drv_timer_t gps_timer = NRF_DRV_TIMER_INSTANCE(2);

static bool fifo_empty;

uint8_t gps_comm(char gps_buffer[NUMBER_MESSAGE_WORD][MESSAGE_BUFFER_SIZE], const char *message_seach_pattern)
{
    uint8_t cr = 0, j = 0, check = 0, i, M = strlen(message_seach_pattern);
    bool check_on = true;

    for (i = 1; i < M; i++)
	check ^= gps_buffer[0][i];

    for (i = 1; i < NUMBER_MESSAGE_WORD; i++)
    {
	do
	    if (app_uart_get(&cr) == NRF_SUCCESS)
	    {
		SEGGER_RTT_printf(0, "%c", cr);

		if (cr == '*')
		{
		    check_on = false;
		    i++;
		    j = 0;
		}

		gps_buffer[i][j++] = cr;

		if (check_on)
		    check ^= cr;
	    }
	while (cr != '\n' && cr != ',' && j < MESSAGE_BUFFER_SIZE + 1);

	j = 0;
	cr = 0;
    }

    char check_buff[3];
    utoa(check, check_buff, 16);
    if (gps_buffer[NUMBER_MESSAGE_WORD - 1][1] != check_buff[0] &&
	gps_buffer[NUMBER_MESSAGE_WORD - 1][2] != check_buff[1])
    {
	lat_direction = long_direction = 'I';
	app_uart_flush();
	return(NRF_ERROR_INVALID_PARAM);
    }

    double raw = strtod (gps_buffer[2], NULL) / 100;
    double frac_degree = fmod(raw, 1) * 100 / 60;
    int32_t lat_temp = (int32_t) (((int32_t)raw + frac_degree) * 10000000);

    raw = strtod (gps_buffer[4], NULL) / 100;
    frac_degree = fmod(raw, 1) * 100 / 60;
    int32_t long_temp = (int32_t) (((int32_t)raw + frac_degree) * 10000000);

    raw = strtod (gps_buffer[9], NULL);
    int32_t alt_temp = (int16_t) (raw * 10);

    if (lat_temp == 0 || long_temp == 0 || alt_temp == 0)
    {
	app_uart_flush();
	return(NRF_ERROR_INVALID_DATA);
    }

    latitude = lat_temp;
    lat_direction = gps_buffer[3][0];
    longitude = long_temp;
    long_direction = gps_buffer[5][0];
    altitude = alt_temp;

    nrf_gpio_pin_clear(GPS_PWR_PIN);
    app_uart_flush();
    return(NRF_SUCCESS);
}


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
        APP_ERROR_HANDLER(p_event->data.error_communication);
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
        APP_ERROR_HANDLER(p_event->data.error_code);
    else if (p_event->evt_type == APP_UART_TX_EMPTY)
	fifo_empty = true;
}

void uart_init(nrf_uart_baudrate_t baud)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          UART_PIN_DISCONNECTED,
          UART_PIN_DISCONNECTED,
          UART_HWFC,
          false,
          baud
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
}

static void gps_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    ret_code_t err_code = bsp_indication_set(BSP_INDICATE_SENT_OK);
    APP_ERROR_CHECK(err_code);

    if (nrf_gpio_pin_out_read(GPS_PWR_PIN))	// If the GPIO pin hasn't been cleared, then a valid message was never received.
	lat_direction = long_direction = 'I';

    nrf_gpio_pin_set(GPS_PWR_PIN);
}

void gps_event_init(void)
{
    ret_code_t err_code;

    nrf_drv_timer_config_t gps_timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    gps_timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    gps_timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    err_code = nrf_drv_timer_init(&gps_timer, &gps_timer_cfg, gps_event_handler);
    APP_ERROR_CHECK(err_code);

    uint32_t cycle_time_sec = *(uint32_t *)GPS_CYCLE_ADDRESS;
    if (cycle_time_sec == ~0)
    {
	cycle_time_sec = 10;
	SEGGER_RTT_WriteString(0, "No valid gps cycle time, set to 10 seconds.\n");
    }

    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&gps_timer, GPS_TIMER * cycle_time_sec);
    nrf_drv_timer_extended_compare(&gps_timer,
				 NRF_TIMER_CC_CHANNEL0,
				 ticks,
				 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
				 true);

    nrf_drv_timer_enable(&gps_timer);
}

static uint32_t message_ack(void)
{
    #define ACK_BUFFER_SIZE 10

    const char ack_search_pattern[] = {0xB5, 0x62, 5};
    uint8_t character, j = 0, ack_buffer[ACK_BUFFER_SIZE], CK_A = 0, CK_B = 0;
    bool first_character = false;

    for (;;)
	while (app_uart_get(&character) == NRF_SUCCESS)
	{
	    SEGGER_RTT_printf(0, "%c", character);
	    if (character == ack_search_pattern[0] && !first_character)
		first_character = true;

	    if (first_character)
		ack_buffer[j++] = character;

	    if (j == ACK_BUFFER_SIZE && ack_buffer[1] == ack_search_pattern[1] && ack_buffer[2] == ack_search_pattern[2])
	    {
		SEGGER_RTT_WriteString(0, "\nAck Pattern found\n");
		if (ack_buffer[3] == 0)
		    return NRF_ERROR_INVALID_FLAGS;

		 for (j = 2; j < ack_buffer[4] + 6; j++)
		 {
		    CK_A += ack_buffer[j];
		    CK_B += CK_A;
		 }

                 if (CK_A == ack_buffer[8] && CK_B == ack_buffer[9])
		    return NRF_SUCCESS;
		 else
		    return NRF_ERROR_INVALID_PARAM;
	    }
	}
}


static uint32_t send_message(message msg)
{
    for (uint8_t i = 0; i < msg.size; i++)
	while (app_uart_put(msg.data[i]) != NRF_SUCCESS);

    return (message_ack());
}


uint32_t gps_config(void)
{
    static const uint8_t port_data[]  = {0xB5, 0x62, 6, 0, 0x14, 0, 1, 0, 0, 0, 0xD0, 8, 0, 0, 0, 0xC2,
				    1, 0, 1, 0, 3, 0, 0, 0, 0, 0, 0xBA, 0x4E};

    static const uint8_t ant_data[]   = {0xB5, 0x62, 6, 0x13, 4, 0, 0, 0, 0xF0, 0x39, 0x46, 0xE6};
    message ant;
    ant.data = ant_data;
    ant.size = (uint8_t const) sizeof(ant_data);

    static const uint8_t inf1_data[]  = {0xB5, 0x62, 6, 2, 0xA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x12, 0xE6};
    message inf1;
    inf1.data = inf1_data;
    inf1.size = (uint8_t const) sizeof(inf1_data);

    static const uint8_t inf2_data[]  = {0xB5, 0x62, 6, 2, 0xA, 0, 1, 0, 0, 0, 7, 0, 0, 7, 7, 0, 0x28, 0x3D};
    message inf2;
    inf2.data = inf2_data;
    inf2.size = (uint8_t const) sizeof(inf2_data);

    static const uint8_t nmea1_data[] = {0xB5, 0x62, 6, 1, 3, 0, 0xF0, 1, 0, 0xFB, 0x11};
    message nmea1;
    nmea1.data = nmea1_data;
    nmea1.size = (uint8_t const) sizeof(nmea1_data);

    static const uint8_t nmea2_data[] = {0xB5, 0x62, 6, 1, 3, 0, 0xF0, 2, 0, 0xFC, 0x13};
    message nmea2;
    nmea2.data = nmea2_data;
    nmea2.size = (uint8_t const) sizeof(nmea2_data);

    static const uint8_t nmea3_data[] = {0xB5, 0x62, 6, 1, 3, 0, 0xF0, 3, 0, 0xFD, 0x15};
    message nmea3;
    nmea3.data = nmea3_data;
    nmea3.size = (uint8_t const) sizeof(nmea3_data);

    static const uint8_t nmea4_data[] = {0xB5, 0x62, 6, 1, 3, 0, 0xF0, 4, 0, 0xFE, 0x17};
    message nmea4;
    nmea4.data = nmea4_data;
    nmea4.size = (uint8_t const) sizeof(nmea4_data);

    static const uint8_t nmea5_data[] = {0xB5, 0x62, 6, 1, 3, 0, 0xF0, 5, 0, 0xFF, 0x19};
    message nmea5;
    nmea5.data = nmea5_data;
    nmea5.size = (uint8_t const) sizeof(nmea5_data);

    static const uint8_t nav5_data[]  = {0xB5, 0x62, 6, 0x24, 0x24, 0, 0xFF, 0xFF, 2, 3, 0, 0, 0, 0, 0x10, 0x27,
				    0, 0, 5, 0, 0xFA, 0, 0xFA, 0, 0x64, 0, 0x5E, 1, 0, 0x3C, 0, 0,
				    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80, 0x80};

    message nav5;
    nav5.data = nav5_data;
    nav5.size = (uint8_t const) sizeof(nav5_data);

    static const uint8_t navx5_data[] = {0xB5, 0x62, 6, 0x23, 0x28, 0, 2, 0, 0xFF, 0xFF, 0x7F, 2, 0, 0, 3, 2,
				    3, 0x20, 6, 0, 0, 1, 0, 0, 0x4B, 7, 0, 1, 0, 0, 1, 1,
				    0, 1, 0, 0x64, 0x64, 0, 0, 1, 0x11, 0, 0, 0, 0, 0, 0x31, 0xDE};

    message navx5;
    navx5.data = navx5_data;
    navx5.size = (uint8_t const) sizeof(navx5_data);

    static const uint8_t pm2_data[]   = {0xB5, 0x62, 6, 0x3B, 0x2C, 0, 1, 6, 0, 0, 0x4E, 0x10, 0x42, 1, 0xE8, 3,
				    0, 0, 0x10, 0x27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x2C, 1,
				    0, 0, 0x4F, 0xC1, 3, 0, 0x86, 2, 0, 0, 0xFE, 0, 0, 0, 0x64, 0x40,
				    1, 0, 0xA2, 0xEA};

    message pm2;
    pm2.data = pm2_data;
    pm2.size = (uint8_t const) sizeof(pm2_data);

    static const uint8_t cfg_data[]   = {0xB5, 0x62, 6, 9, 0xC, 0, 0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0x17, 0x75};
    message cfg;
    cfg.data = cfg_data;
    cfg.size = (uint8_t const) sizeof(cfg_data);

    const message *msg[] = {&ant, &inf1, &inf2, &nmea1, &nmea2, &nmea3, &nmea4, &nmea5, &nav5, &navx5, &pm2, &cfg};

    for (uint8_t i = 0; i < sizeof(port_data); i++)
        while (app_uart_put(port_data[i]) != NRF_SUCCESS);

    while (!fifo_empty);
    fifo_empty = false;
    app_uart_close();
    uart_init(NRF_UART_BAUDRATE_115200);

    if (message_ack() != NRF_SUCCESS)
	return NRF_ERROR_INTERNAL;

    for (uint8_t i = 0; i < (sizeof(msg) / 4); i++)
	if (send_message(*msg[i]) != NRF_SUCCESS);
	    return NRF_ERROR_INTERNAL;

    return NRF_SUCCESS;
}
