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

#define LED_COUNT 8
#define LED_PIN 10
#define IOSTB_PIN 15

WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Read bus on pio0

static PIO sPioReadData = pio0;
static int sSmRead = -1;
static int sSmReadBusOffset = -1;

//------------------------------------------------------------------------------

bool pio_init()
{
    // Input/Blanking on PIO 0

    // Find a place for the PIO program in the instruction memory
    sSmReadBusOffset = pio_add_program(sPioReadData, &read_bus_data_program);
    // Claim an unused state machine for the column reading and run the program
    sSmRead = pio_claim_unused_sm(sPioReadData, true);
    read_bus_data_program_init(sPioReadData, sSmRead, sSmReadBusOffset, IOSTB_PIN);
  
    return ((sSmRead != -1));
}

uint16_t pio_read_bus_data()
{
    // Process all new data
    bool noBusData = false;
    uint8_t numBusData = 0;
        bool waited = false;
    while (!noBuslData )
    {
        // Check for new row data
        if (sSmRow != -1)
        {
            noRowData = pio_sm_is_rx_fifo_empty(sPioColRow, sSmRow);
            if (!noRowData)
            {
                uint32_t d = pio_sm_get(sPioColRow, sSmRow);
                sRowDataPrel = (uint8_t)((d >> 23) & 0xff);
                numRowData++;
            }
        }

            // wait a bit for new data to arrive - all input should happen
        // within a few microseconds
        if (!waited && noColData && noRowData && ((numColData + numRowData) < 4))
        {
            waited = true;
            noColData = false;
            sleep_us(6);
        }
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

void setup() {

    Serial.begin(115200);

    ws2812fx.init();
    ws2812fx.setBrightness(100);
    ws2812fx.setSpeed(200);
    ws2812fx.setMode(FX_MODE_CHASE_FLASH);
    ws2812fx.start();
  
    delay(500);
    Serial.printf("Starting\n");

     // PIO setup

    if (!pio_init())
    {
        Serial.printf("Failed to initialize the PIOs!\n");
        panic_mode();
    } else {  ws2812fx.setMode(FX_MODE_COMET); }

}

void loop() {
    
    ws2812fx.service();
    Serial.printf("Looping\n");

}
