/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_long_range_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon main file.
 *
 * This file contains the source code for long range beacon application.
 */

#include "app_timer.h"
#include "ble_advdata.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_pwr_mgmt.h"

#include "ble_gap.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <nrf_assert.h>
#include "pa_lna.h"
#include "app_uart.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "SEGGER_RTT.h"

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */

//#define NON_CONNECTABLE_ADV_INTERVAL MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_COMPANY_IDENTIFIER 0x0059 /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_DEVICE_TYPE 2          /**< 0x02 refers to Beacon. */
#define APP_ADV_DATA_LENGTH 25      /**< Length of manufacturer specific data in the advertisement. */

#define APP_BEACON_INFO_LENGTH APP_ADV_DATA_LENGTH + 2 /**< Total length of information advertised by the Beacon. */

#define APP_VERSION 2
#define APP_BEACON_UUID 0, 0, 0		/**< Customer Information */

#define APP_BEACON_MAC 0, 0, 0, 0, 0, 0 /**< Placeholder for Device Address */

#define APP_LAT_VALUE 0, 0, 0, 0	/**< Placeholder for Latitude */
#define APP_LAT_DIR 0			/**< Placeholder for Latitude Direction */
#define APP_LONG_VALUE 0, 0, 0, 0	/**< Placeholder for Longitude */
#define APP_LONG_DIR 0			/**< Placeholder for Longitude Direction */
#define APP_ALT_VALUE 0, 0		/**< Placeholder for Altitude */

#define APP_BAT_VALUE 0, 0		/**< Placeholder for Battery Voltage */

#define APP_MEASURED_RSSI 0xC3 /**< The Beacon's measured RSSI at 1 meter distance in dBm. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UUIC_OFFSET_IN_BEACON_INFO 3                              /**< Position of the beginning of the UUIC */
#define DATA_OFFSET_IN_BEACON_INFO UUIC_OFFSET_IN_BEACON_INFO + 9 /**< Position of the beginning of variable data */
#define UICR_ADDRESS 0x10001080                                  /**< Address of the UICR register. */
#define ADVERTISING_CYCLE_ADDRESS 0x10001084			/**< Address of the cycle time values for advertising. */
#define GPS_CYCLE_ADDRESS 0x10001088                            /**< Address of the cycle time values for GPS. */

#define SAMPLES_IN_BUFFER 3				// Change this value only if necessary. Leave the timers as is.
#define BATTERY_TIMER MSEC_TO_UNITS(10000, UNIT_10_MS) / SAMPLES_IN_BUFFER  // The time is multiplied by the SAMPLES_IN_BUFFER
#define GPS_TIMER MSEC_TO_UNITS(10000, UNIT_10_MS)	// This is in terms of 100uS. Both set are to 1 sec and multiplied by the user value.

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define NUMBER_MESSAGE_WORD 16
#define MESSAGE_BUFFER_SIZE 12

#define DBM 8 // Supported tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm, +8dBm.

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

static const nrf_drv_timer_t gps_timer = NRF_DRV_TIMER_INSTANCE(2);

static ble_gap_addr_t           addr; // Log our BLE address (6 bytes).
static ble_advdata_t            advdata;
static ble_advdata_manuf_data_t manuf_specific_data;
static ble_gap_adv_params_t     m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t                  m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t                  m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
static const char * message_seach_pattern = "$GxGGA,";
static char gps_buffer[NUMBER_MESSAGE_WORD][MESSAGE_BUFFER_SIZE];

static bool use_coded;
typedef enum {PHY_1M, PHY_CODED, PHY_BOTH} phyType;
static phyType phy_used;

static int32_t latitude, longitude;
static char lat_direction, long_direction;
static int16_t altitude;


/**@brief Struct that contains pointers to the encoded advertising data. */

static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
	.p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
	.p_data = NULL,
	.len    = 0
    }
};


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] = /**< Information advertised by the Beacon. */
{
        APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                             // implementation.
        APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                             // manufacturer specific data in this implementation.
        APP_VERSION,         // Beacon packet set version
        APP_BEACON_UUID,     // 3 byte UUID value.
        APP_BEACON_MAC,      // Device address
	APP_LAT_VALUE,	    // Latitude Value
	APP_LAT_DIR,	    // Latitude Direction
	APP_LONG_VALUE,	    // Longitude Value
	APP_LONG_DIR,	    // Longitude Direction
	APP_ALT_VALUE,	    // Altitude Value
        APP_BAT_VALUE,       // Battery Voltage
        APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                             // this implementation.
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


static uint8_t gps_comm(void)
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


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void change_manuf_specific_data(uint16_t battery_voltage)
{
    uint32_t err_code;
    uint8_t index = DATA_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_32(latitude);
    m_beacon_info[index++] = (latitude & 0x00FF0000) >> 16;
    m_beacon_info[index++] = (latitude & 0x0000FF00) >> 8;
    m_beacon_info[index++] = LSB_32(latitude);
    m_beacon_info[index++] = lat_direction;
    m_beacon_info[index++] = MSB_32(longitude);
    m_beacon_info[index++] = (longitude & 0x00FF0000) >> 16;
    m_beacon_info[index++] = (longitude & 0x0000FF00) >> 8;
    m_beacon_info[index++] = LSB_32(longitude);
    m_beacon_info[index++] = long_direction;
    m_beacon_info[index++] = MSB_16(altitude);
    m_beacon_info[index++] = LSB_16(altitude);

    m_beacon_info[index++] = MSB_16(battery_voltage);
    m_beacon_info[index]   = LSB_16(battery_voltage);

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    advdata.p_manuf_specific_data = &manuf_specific_data;

    if (phy_used == PHY_BOTH)
    {
	if (use_coded)
	{
	    m_adv_params.primary_phy     = BLE_GAP_PHY_CODED;
	    m_adv_params.secondary_phy   = BLE_GAP_PHY_CODED;
	    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
	    use_coded                    = false;
	}
	else
	{
	    m_adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
	    m_adv_params.secondary_phy   = BLE_GAP_PHY_1MBPS;
	    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
	    use_coded                    = true;
	}
    }

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, DBM);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_start(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    uint8_t index = UUIC_OFFSET_IN_BEACON_INFO;

    phy_used = MSB_32(*(uint32_t *)UICR_ADDRESS);

    if (phy_used > PHY_BOTH)
    {
	SEGGER_RTT_WriteString(0, "No valid PHY, set to 1M.\n");
	phy_used = PHY_1M;
    }

    m_beacon_info[index++] = ((*(uint32_t *)UICR_ADDRESS) & 0x00FF0000) >> 16;
    m_beacon_info[index++] = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FF00) >> 8;
    m_beacon_info[index++] = LSB_32(*(uint32_t *)UICR_ADDRESS);

    m_beacon_info[index++] = addr.addr[5];
    m_beacon_info[index++] = addr.addr[4];
    m_beacon_info[index++] = addr.addr[3];
    m_beacon_info[index++] = addr.addr[2];
    m_beacon_info[index++] = addr.addr[1];
    m_beacon_info[index]   = addr.addr[0];

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.scan_req_notification = 0;
    m_adv_params.p_peer_addr           = NULL; // Undirected advertisement.
    m_adv_params.filter_policy         = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval              = BLE_GAP_ADV_INTERVAL_MAX;
    m_adv_params.duration              = 0; // Never time out.
    m_adv_params.max_adv_evts          = 1;

    if (phy_used == PHY_CODED)
    {
	m_adv_params.primary_phy     = BLE_GAP_PHY_CODED;
	m_adv_params.secondary_phy   = BLE_GAP_PHY_CODED;
	m_adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    }
    else if (phy_used == PHY_1M)
    {
	m_adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
	m_adv_params.secondary_phy   = BLE_GAP_PHY_1MBPS;
	m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    }

    change_manuf_specific_data(0xFFFF);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code           = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
	nrf_pwr_mgmt_run();
}

static void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
// Dummy handler for battery/saadc timer. Event is handled in saadc_callback().
}

static void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    uint32_t cycle_time_sec = *(uint32_t *)ADVERTISING_CYCLE_ADDRESS;
    if (cycle_time_sec == ~0)
    {
	cycle_time_sec = 1;
	SEGGER_RTT_WriteString(0, "No valid advertising cycle time, set to 1 second.\n");
    }

    /* setup m_timer for compare event */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, BATTERY_TIMER * cycle_time_sec);
    nrf_drv_timer_extended_compare(&m_timer,
				 NRF_TIMER_CC_CHANNEL0,
				 ticks,
				 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
				 false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
									      NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
					timer_compare_event_addr,
					saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

static void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

static void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
	ret_code_t err_code;
	uint32_t battery_voltage, average_sample = 0;
	err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
	APP_ERROR_CHECK(err_code);

	SEGGER_RTT_printf(0, "\nADC event number: %d\n", m_adc_evt_counter);

	for (uint8_t i = 0; i < SAMPLES_IN_BUFFER; i++)
	{
	    battery_voltage = p_event->data.done.p_buffer[i] * 3600 / 1024;

	    SEGGER_RTT_printf(0, "%d\n", battery_voltage);
	    average_sample += battery_voltage;
	}

	m_adc_evt_counter++;

	average_sample /= SAMPLES_IN_BUFFER;

	change_manuf_specific_data((uint16_t) average_sample);
    }
}

static void saadc_init(void)
{
    ret_code_t                 err_code;
    nrf_saadc_channel_config_t channel_config =
    //       NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for getting the device address.
 */
static void ble_log_mac_address(void)
{
    uint32_t err_code = sd_ble_gap_addr_get(&addr);
    APP_ERROR_CHECK(err_code);

    SEGGER_RTT_printf(0, "\n%02X:%02X:%02X:%02X:%02X:%02X\n",
		    addr.addr[0], addr.addr[1],
		    addr.addr[2], addr.addr[3],
		    addr.addr[4], addr.addr[5]);
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
        APP_ERROR_HANDLER(p_event->data.error_communication);
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
        APP_ERROR_HANDLER(p_event->data.error_code);
}

static void uart_init(void)
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
          NRF_UART_BAUDRATE_115200
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

static void gps_event_init(void)
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

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint8_t cr, j = 0, M = strlen(message_seach_pattern);
    bool first_char = false;

    sd_power_dcdc_mode_set(true);
    log_init();
    timers_init();
    leds_init();
    power_management_init();
    saadc_init();
    saadc_sampling_event_init();
    ble_stack_init();
    ble_log_mac_address();
    gps_event_init();
    pa_lna_init(APP_PA_PIN, APP_LNA_PIN);

    advertising_start();
    SEGGER_RTT_WriteString(0, "Beacon started.\n");

    nrf_gpio_cfg_output(GPS_PWR_PIN);
    uart_init();
    nrf_gpio_pin_set(GPS_PWR_PIN);

    saadc_sampling_event_enable();
    SEGGER_RTT_WriteString(0, "SAADC started.\n");

    // Enter main loop.
    for (;;)
    {
	while (app_uart_get(&cr) == NRF_SUCCESS)
	{
	    SEGGER_RTT_printf(0, "%c", cr);
	    if (cr == message_seach_pattern[0])
		first_char = true;

	    if (first_char)
		gps_buffer[0][j++] = cr;

	    if (j == M)
	    {
		for (j = 3; j < M; j++)	// We have a good first character and we skip the third as it could vary.
		    if (gps_buffer[0][j] != message_seach_pattern[j])
			break;

		if (j == M)
		{
		    SEGGER_RTT_WriteString(0, "\nPattern found\n");

		    if (gps_comm() == NRF_SUCCESS)
			SEGGER_RTT_WriteString(0, "Message Loaded\n");
		    else
			SEGGER_RTT_WriteString(0, "Message Error\n");
		}

		j = 0;
		first_char = false;
	    }
	}

	idle_state_handle();
    }
}