/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// PIO logic analyser example
//
// This program captures samples from a group of pins, at a fixed rate, once a
// trigger condition is detected (level condition on one pin). The samples are
// transferred to a capture buffer using the system DMA.
//
// 1 to 32 pins can be captured, at a sample rate no greater than system clock
// frequency.

#include <Arduino.h>
#include <WS2812FX.h>

#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"

#include "read_bus_data.pio.h"

// Some logic to analyse:
#include "hardware/structs/pwm.h"

#define LED_COUNT 35
#define LED_PIN 10
#define IOSTB_PIN 15

#define PICO_DEFAULT_LED_PIN 16
#define SAM_DETECTOR_VERSION 100


WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//------------------------------------------------------------------------------
// Local data

// Read bus on pio0
static PIO sPioReadData = pio0;

static int sSmRead = -1;
static int sSmReadBusOffset = -1;

static uint8_t numBusData = 0;
static uint32_t bus_data = 0;

static uint8_t data = 0;
static uint8_t adress = 0;

static char msg[32];

//------------------------------------------------------------------------------

bool pio_init()
{
    // Input on PIO0

    // Find a place for the PIO program in the instruction memory
    sSmReadBusOffset = pio_add_program(sPioReadData, &read_bus_data_program);
    // Claim an unused state machine for data reading and run the program
    sSmRead = pio_claim_unused_sm(sPioReadData, true);
    read_bus_data_program_init(sPioReadData, sSmRead, sSmReadBusOffset, IOSTB_PIN);
  
    return ((sSmRead != -1));
}

uint32_t pio_read_bus_data()
{
    // Process all new data
    
    bool noBusData = false;
    uint32_t sBusData = 1 ;


    if (sSmRead != -1)
        {
            // noBusData = pio_sm_is_rx_fifo_empty(sPioReadData, sSmRead);

            sBusData = pio_sm_get_blocking(sPioReadData, sSmRead);

        }
        
    return (sBusData);
}

void panic_mode()
{
    // endless panic
    while (true)
    {
        // blinking alert
        ws2812fx.service();
    }
}

void sam_init()
{
    Serial.printf("SAM Detector \n", SAM_DETECTOR_VERSION);
    
}

uint32_t sam_process_data() {

    // read data from PIO
    uint32_t inData = pio_read_bus_data();
    return(inData);

}

void setup() {

    Serial.begin(400000);

    ws2812fx.init();
    ws2812fx.setBrightness(100);
    ws2812fx.setSpeed(200);
    ws2812fx.setMode(FX_MODE_CHASE_FLASH);
    ws2812fx.start();

    delay(500);
    Serial.printf("\nStern SAM Detector v0.1\n");

    // initialize the Pi Pico's LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // PIO setup
    if (!pio_init())
    {
        Serial.printf("Failed to initialize the PIOs!\n");
        panic_mode();
    }

    sam_init();

}

void loop() {
    
    bus_data = sam_process_data();

    data = bus_data & 0xFF;
    adress = (bus_data >> 26) & 0xF;

    if (bus_data != 1)
        {
        Serial.println(bus_data);
        // Serial.printf("Adress=");
        // Serial.println(adress);
        // Serial.printf("Data=");
        // Serial.println(data);
        }
       
    ws2812fx.service();
    
}
