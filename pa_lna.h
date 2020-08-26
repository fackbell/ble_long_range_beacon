#ifndef PA_LNA_H__
#define PA_LNA_H__

#include "ble.h"
#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"

void pa_lna_init(uint32_t gpio_pa_pin, uint32_t gpio_lna_pin);

#endif
