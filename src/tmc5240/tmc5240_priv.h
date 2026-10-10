/* Internals shared inside the component. Not part of the public API. */
#pragma once

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

#include "stepper_priv.h"
#include "tmc5240.h"

struct tmc5240_t {
    struct stepper_dev stepper; /* the category view; priv points back here */
    struct stepper_bus bus;
#ifdef ESP_PLATFORM
    SemaphoreHandle_t lock; /* one write, or one whole read chain, at a time */
#endif
    struct stepper_pin en;  /* DRV_ENN, active low */

    double  fclk;          /* what the ramp registers are counted in */
    double  rref;          /* kOhm on IREF, needed to recompute current later */
    double  range_rms;     /* full-scale RMS current of the selected range */
    float   run_rms;       /* what CS actually delivers, not what was asked */
    int     microsteps;
    uint32_t vmax_reg;     /* the ramp's VMAX; run() and stop() overwrite the register */
    uint32_t amax_reg;     /* saved so stop_hard() can borrow AMAX and give it back */
    bool    amax_borrowed; /* AMAX still holds stop_hard()'s maximum */
    bool    positioning;   /* last command was move_to, so XTARGET is live */
    uint8_t spi_status;    /* bits 39:32 of the last datagram */
};
