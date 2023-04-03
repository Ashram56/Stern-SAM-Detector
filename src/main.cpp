/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// PIO logic analyser example
// Stern SAM Detector

// Generic Stern SAM bus analyzis
// Extracted from schematics reverse engineering
// Main github : https://github.com/Ashram56/Stern-SAM-Databus-Analysis

// A3 A2 A1 A0 = SOL A / SOL B / SOL C / FLSH_LMP / NA / STATUS / AUX_DRV / AUX_IN / LMP_STB / AUX_LMP / LMP_DRV / Clk_input_aux

//  0 = 0000 = 1 0 0 0 0 0 0 0 0 0 0 0 = SOL_A
//  1 = 0001 = 0 1 0 0 0 0 0 0 0 0 0 0 = SOL_B
//  2 = 0010 = 0 0 1 0 0 0 0 0 0 0 0 0 = SOL_C
//  3 = 0011 = 0 0 0 1 0 0 0 0 0 0 0 0 = FLSH_LMP
//  4 = 0100 = 0 0 0 0 1 0 0 0 0 0 0 0 = NA
//  5 = 0101 = 0 0 0 0 0 1 0 0 0 0 0 0 = STATUS
//  6 = 0110 = 0 0 0 0 0 0 1 0 0 0 0 0 = AUX_DRV
//  7 = 0111 = 0 0 0 0 0 0 0 1 0 0 0 0 = AUX_IN
//  8 = 1000 = 0 0 0 0 0 0 0 0 1 0 0 0 = LMP_STB
//  9 = 1001 = 0 0 0 0 0 0 0 0 0 1 0 0 = AUX_LMP
// 10 = 1010 = 0 0 0 0 0 0 0 0 0 0 1 0 = LMP_DRV
// 11 = 1011 = 0 0 0 0 0 0 0 0 0 0 0 1 = Clk_input_aux

// J6 = FLSH_LMP
// J7 = SOL_C
// J8 = SOL_B
// J9 = SOL_A
// J12 = LMP_STB
// J13 = LMP_DRV
// J15 = RLY_DRV = General illumination

// Transistor assignment (not linear)
// see https://github.com/Ashram56/Stern-SAM-Databus-Analysis/blob/main/Transistor%20to%20pin%20assignment

// Specifically for Tron

// Disc motor relay = Q30 = D5 / FLSH_LMP
// Disc motor direction = Q22 = D5 / SOL_C


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

#define LED_COUNT 1
#define LED_PIN 16
#define IOSTB_PIN 15

// #define PICO_DEFAULT_LED_PIN 16
#define SAM_DETECTOR_VERSION 100


WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); // on RPI Pico Zero led is a WS2812 led

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

static uint8_t disc_on = 0;
static uint8_t disc_relaydir = 0;

static uint8_t enable_mask = 0;
static uint8_t masked_adress = 0 ;

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

pinMode (0,INPUT);
pinMode (1,INPUT);
pinMode (2,INPUT);
pinMode (3,INPUT);
pinMode (4,INPUT);
pinMode (5,INPUT);
pinMode (6,INPUT);
pinMode (7,INPUT);

pinMode (14,INPUT);
pinMode (15,INPUT);

pinMode (26,INPUT);
pinMode (27,INPUT);
pinMode (28,INPUT);
pinMode (29,INPUT);


}

uint32_t sam_process_data() {

    // read data from PIO
    uint32_t inData = pio_read_bus_data();
    return(inData);

}


void setup() {

    Serial.begin(400000);

    delay(500);
    Serial.printf("\nStern SAM Detector v0.1\n");

    // PIO setup
    if (!pio_init())
    {
        Serial.printf("Failed to initialize the PIOs!\n");
        panic_mode();
    }
 
    sam_init();

    ws2812fx.init();
    ws2812fx.setBrightness(50);
    ws2812fx.setSpeed(200);
    ws2812fx.setMode(FX_MODE_CHASE_FLASH);
    ws2812fx.start();

}

void loop() {
    

    bus_data = sam_process_data();

    data = bus_data & 0xFF;             // get data from bits 0-7
    adress = (bus_data >> 26) & 0xF;    // get adress from bits 26-29


    if (data != 0)
    {
            switch (adress) 

            {

            case 0:
            Serial.printf("SOL_A=");
            Serial.println(data);
            break;

            case 1:
            Serial.printf("SOL_B=");
            Serial.println(data);
            disc_relaydir = (data >>2) & 0x1 ;
            break;

            case 2:
            Serial.printf("SOL_C=");
            Serial.println(data);
            break;

            case 3:
            Serial.printf("LMP_FLSH=");
            Serial.println(data);
            disc_on = (data >>5) & 0x1 ;
            break;

            case 6:
            Serial.printf("AUX_DRV=");
            Serial.println(data);
            break;

              
//            case 8:
//            Serial.printf("LMP_STB=");
//            Serial.println(data);
//            break;

//            case 10:
//            Serial.printf("LMP_DRV=");
//            Serial.println(data);
//            break;

            }

    }

    if (disc_on=1)
    {
    Serial.printf("Disc Status=");
    Serial.print(disc_on);
    Serial.println(disc_relaydir);
    }


}
         



void setup1()
{

}

void loop1()

{
    ws2812fx.service();

 }
