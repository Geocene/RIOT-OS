/*
 * Copyright (C) 2017 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_ekmb
 * @{
 *
 * @file
 * @brief       Driver for the EKMB PIR Motion Sensor.
 *
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 *
 * @}
 */

// This driver returns the duty cycle of the motion sensor through the ekmb_read callback.
// Duty cycle is provided as an integer between 0 and MAX_RVAL. So as an example, if MAX_RVAL is 1000
// and ekmb_read gives you 200, that means the detection pin was high for 20% of the time between now
// and the last ekmb_read call.
// 
// This driver is interrupt driven and counts ticks from the xtimer to determine duty cycle. If you raise
// MAX_RVAL, the xtimer tick frequency, or the time between ekmb_read calls, be careful not to overflow the
// 32 bit integers we're using for math.

#include <string.h>

#include "ekmb.h"
#include "xtimer.h"

// When the motion sensor detects motion, turns on the LED (and vice versa)
#define MOTION_TURNS_ON_LED (0)
// Record the tick value at which high and low detection edges arrive for examination by debugger
#define RECORD_EDGES    (0)

#define ENABLE_DEBUG    (0)
#include "debug.h"

#if MOTION_TURNS_ON_LED
#include "board.h"
#endif


static bool     pin_high       = false; // Value of the pin after the most recent edge
static uint32_t pin_rise_time  = 0;     // Time of the most recent rising edge (ticks)
static uint32_t accum_up_time  = 0;     // Accumulated on time since the last ekmb_read call (ticks)
static uint32_t last_read_time = 0;     // Time of last ekmb_read call (ticks)
static const uint32_t MAX_RVAL = 1000;  // Maximum value returned by ekmb_read.

#if RECORD_EDGES
#define EDGE_BUF_LEN 20
static volatile uint32_t period_start = 0;
static volatile uint32_t rising_edges[EDGE_BUF_LEN] = {0};
static volatile uint32_t rising_edges_i = 0;
static volatile uint32_t falling_edges[EDGE_BUF_LEN] = {0};
static volatile uint32_t falling_edges_i = 0;
#endif

static uint32_t getTicks(void) {
    xtimer_ticks32_t now = xtimer_now();
    return now.ticks32;
}

void ekmb_trigger(void* arg) {
    ekmb_t* dev = (ekmb_t*) arg;
    int pin_now = gpio_read(dev->p.gpio);
    uint32_t now = getTicks();

    if (pin_now) { 
        // We caught a rising edge
        pin_rise_time = now;
#if RECORD_EDGES
        if(rising_edges_i < EDGE_BUF_LEN) {
            rising_edges[rising_edges_i++] = now - period_start;
        }
#endif
#if MOTION_TURNS_ON_LED
        LED_ON;
#endif
    } else {
        // We caught a falling edge
        accum_up_time += (now - pin_rise_time);
#if RECORD_EDGES
        if(falling_edges_i < EDGE_BUF_LEN) {
            falling_edges[falling_edges_i++] = now - period_start;
        }
#endif
#if MOTION_TURNS_ON_LED
        LED_OFF;
#endif
    }
    pin_high = pin_now;
}

int ekmb_init(ekmb_t *dev, const ekmb_params_t *params) {
    dev->p.gpio = params->gpio;
    uint32_t now = getTicks();
#if RECORD_EDGES
    period_start = now;
#endif
    last_read_time = now;
    if (gpio_init_int(params->gpio, GPIO_IN_PD, GPIO_BOTH, ekmb_trigger, dev)) {
        return -1;
    }
    return 0;
}

int ekmb_read(ekmb_t *dev, int16_t *occup) {
    uint32_t now = getTicks();

    if(pin_high) {
        accum_up_time += (now - pin_rise_time);
        pin_rise_time = now;
    }

    uint32_t num = accum_up_time * MAX_RVAL;
    uint32_t denom = now - last_read_time;
    uint32_t rval = num / denom;
    rval = rval > MAX_RVAL ? MAX_RVAL : rval;
    *((uint16_t*)occup) = rval;
    
    last_read_time = now;
    accum_up_time = 0;

#if RECORD_EDGES
    period_start = now;
    for(uint32_t i = 0; i < rising_edges_i; i++) {
        rising_edges[i] = 0;
    }
    for(uint32_t i = 0; i < falling_edges_i; i++) {
        falling_edges[i] = 0;
    }
    rising_edges_i = 0;
    falling_edges_i = 0;
#endif

    return 0;
}

