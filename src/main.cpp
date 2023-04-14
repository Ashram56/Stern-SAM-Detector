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

#include <iostream>
#include <vector>
#include <bitset>
#include <array>

using namespace std;

#define LED_COUNT 1
#define LED_PIN 16
#define IOSTB_PIN 15
#define TIME_ROW 40 // define across how many cycle we calculate the duty cycle - Calculation is performed every time the full matrix is updated, which is every 12ms. So total time calculation = 12ms * TIME_ROW 

// #define PICO_DEFAULT_LED_PIN 16
#define SAM_DETECTOR_VERSION 100

// #define DEBUG_LOG // Display various stage of matrix processing
// #define DEBUG_COUNTER // Display on serial output the process of the counting function
#define DEBUG_LAMP_COUNTER // Display 80 byte vector with count, every TIME_ROW
// #define DEBUG_LAMP_MATRIX


WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); // on RPI Pico Zero led is a WS2812 led

//------------------------------------------------------------------------------
// Local data

// Read bus on pio0
static PIO sPioReadData = pio0;

static int sSmRead = -1;
static int sSmReadBusOffset = -1;

static uint8_t numBusData = 0;
static uint32_t bus_data = 0;

static uint8_t decoded_data = 0;
static uint8_t decoded_adress = 0;

static uint8_t disc_on = 0;
static uint8_t disc_relaydir = 0;

static uint8_t enable_mask = 0;
static uint8_t masked_adress = 0 ;
static uint32_t timestamp_strobe = 0 ;
static uint32_t timestamp_drv = 0 ;
static uint32_t timestamp_aux = 0 ;
static uint32_t timestamp = 0 ;

static char msg[32];

static uint8_t matrix[TIME_ROW][10] = {0};
static uint8_t lamp_state[10][8] = {0};
static uint16_t index_drv = 0;
static uint8_t aux_lamp = 0x0;
static uint16_t lamp_strobe = 0x0;
static uint8_t lamp_drive = 0x0;
static uint8_t threshold_zero = 2;
static uint8_t threshold_one = 8;

static vector<int8_t> lampSum(80, 0); // Define a global vector to store the sum of each hardware signal

static bool lamp_matrix[10][8]={0}; // global matrix containing lamp matrix state


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

timestamp_strobe=micros();
timestamp_drv = micros();
timestamp= micros();

}

uint32_t sam_process_data() {

    // read data from PIO
    uint32_t inData = pio_read_bus_data();
    return(inData);

}

// lamp detection logic 

// Function to calculate logical AND between each bit in the 10-bit variable and each bit in the 8-bit variable
// update ONLY the row defined by index_stb - that's because the row switches every ms to the next row, only one active row at a time.
// We want to keep the rest of the matrix to reflect the overall matrix state

void calculateAndMatrix(unsigned int bit10, unsigned char bit8, bool matrix[10][8])
{
    // Convert bit10 to bitset for easier manipulation of its bits
    bitset<10> bitset10(bit10);

    // calculate bit position in bit10. This reflects the active row
    
    int reg = bit10;
    int pos = 0;
    while (reg != 0) {
        if (reg & 1) {
            break;;
        }
        pos++;
        reg >>= 1;
    }
    
    // Loop through each bit in the inverted 8-bit variable
    for (int j = 7; j >= 0; j--) {
           
        // Calculate logical AND between corresponding bits in row and column
        matrix[pos][7-j] = bitset10[pos] && ((bit8 >> j) & 1);
        }
}
    

// Function that will convert a 10x8 boolean matrix into a 10 byte vector, each byte reflecting one row of the input matrix (8 bit = 1 byte)

std::vector<uint8_t> matrixToVector(bool matrix[10][8]) {
    std::vector<uint8_t> result(10, 0); // initialize the result vector with 10 zero bytes
    
    for (int i = 0; i < 10; i++) {
        std::bitset<8> row;
        for (int j = 0; j < 8; j++) {
            
            if (matrix[i][j]) {
                row.set(7-j);
            }
                   
        }
        
        result[i] = static_cast<uint8_t>(row.to_ulong());
     
    }

    return result;
}


// Function to convert a 10 byte vector where each byte in vector reflect the row, and the byte position in the vector reflect the column
// A TIME_ROWx10 matrix is used to store this data, and the corresponding row is modified accordingly to the index parameter

void convert_vector(std::vector<uint8_t>& vec, uint8_t matrix[TIME_ROW][10], int index) {
    
    for (int i = 0; i < 10; i++) {
        matrix[index][i] = vec[i];
    
    } 
}

// count 0 and 1 in TIME_ROWx10 matrix
// Each cell in the matrix is a byte and a binary physical representation of a series of hardware signal, one byte equals 8 signals, for a total of 80 signals for a single row.
// Row is time, column defines multiple HW blocks. I need to sum the number of 1 and the number of zero in the matrix for each hardware signal across all TIME_ROW rows

    vector<int8_t> countSignals(uint8_t matrix[TIME_ROW][10]) {
    vector<int8_t> signalSums(80, 0);

    // Iterate over each row and column in the matrix
    for (int row = 0; row < TIME_ROW; row++) {
        for (int col = 0; col < 10; col++) {
            // Convert the byte to a binary string
            string binaryString = bitset<8>(matrix[row][col]).to_string();

            // Iterate over each character in the binary string
            for (int i = 0; i < 8; i++) {
                // Get the index of the hardware signal corresponding to the current bit
                int signalIndex = col * 8 + i;

                // Increment the count for the corresponding element in the array
                // int8_t& signalSum = signalSums[signalIndex];
                // signalSum += binaryString[i] - '0';

                // trick to convert 0/1 defined as char value into a numerical value
                signalSums[signalIndex]+=binaryString[i]-'0';

                #ifdef DEBUG_COUNTER

                Serial.print("Digit extract=");
                Serial.println(binaryString[i]);

                Serial.print("Signal Index=");
                Serial.print(signalIndex);
                Serial.print(" ");
                Serial.println(signalSums[signalIndex]);

                #endif

            }
        }
    }

    // Return the array containing the sum of each hardware signal over all TIME_ROW rows
    return signalSums;
}

// Main function to convert STB and DRV bytes into a lamp matrix, then use this matrix to convert to a TIME_ROWx10 matrix where each row is defined by the index_drv value

void update_matrix(uint16_t lamp_strobe, uint8_t lamp_drive, bool lamp_matrix[10][8]) {

std::vector<uint8_t> lamp_vector;

// calculate lamp_matrix using lamp_strobe and lamp_drive as input
calculateAndMatrix(lamp_strobe,lamp_drive, lamp_matrix);

if (lamp_strobe==512) 
    {

        #ifdef DEBUG_LAMP_MATRIX

            Serial.println ("==");

                for(int i = 0; i < 10; i++) {
                for(int j = 0; j < 8; j++) {
                    Serial.print(lamp_matrix[i][j]);
                    Serial.print(" ");
                }
            Serial.println();
            }
        #endif

    // convert matrix into 10 byte vector
    lamp_vector = matrixToVector(lamp_matrix);

        #ifdef DEBUG_LOG

        for(int i = 0; i < 10; i++) {
        Serial.print(lamp_vector[i],BIN);
        Serial.print(" ");
            }

        Serial.println();

        #endif

    // fill TIME_ROWx10 matrix row according to index_drv with lamp vector
    convert_vector(lamp_vector,matrix,index_drv);
    index_drv++;

        #ifdef DEBUG_LOG

        Serial.print("Vector in matrix=");
        Serial.println(index_drv);
        for(int i = 0; i < 10; i++) {
            Serial.print(matrix[index_drv][i],BIN);
            Serial.print(" ");
            }

        Serial.println();

        #endif

    }

}

// Function below will concatenate two registers, this is to build lamp_strobe variable which covers both LMP_STB and AUX_LAMP, resp 8b and 2b
// register2 is aux_lamp
// register1 is lamp_strobe, but we need to mask bit 8 and 9 before applying mask

uint16_t concatenateRegisters(uint16_t register1, uint8_t register2) {
    // Mask the first two bits of register2 and shift them to the left
    uint16_t maskedAndShiftedRegister2 = (static_cast<uint16_t>(register2 & 0x03) << 8);

    uint16_t maskedRegister1 = register1 & 0x0F;
 
    // Combine the maskedAndShiftedRegister2 with register1 and maskedRegister2
    uint16_t concatenatedValue = (maskedAndShiftedRegister2) | (maskedRegister1);

    return concatenatedValue;
}

uint32_t check_timestamp(uint32_t stamp)

{
    Serial.println(micros()-stamp);
    stamp=micros();
    return stamp;
}


void check_index() {

    if (index_drv==TIME_ROW-1) { 
            lampSum=countSignals(matrix);
           
            index_drv=0;

            #ifdef DEBUG_LAMP_COUNTER

                Serial.println();
                Serial.println("Lamp counter = ");
                for (int i=0;i<80;i++) {
                    Serial.print(lampSum[i],DEC);
                    Serial.print(" ");
                }
                Serial.println();               

            #endif
    }

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

    decoded_data = bus_data & 0xFF;             // get data from bits 0-7
    decoded_adress = (bus_data >> 26) & 0xF;    // get adress from bits 26-29

    if (decoded_data != 0) {

            switch (decoded_adress) 

            {

            case 0:
        //    Serial.printf("SOL_A=");
        //    Serial.println(decoded_data);
            break;

            case 1:
        //    Serial.printf("SOL_B=");
        //    Serial.println(decoded_data);
            break;

            case 2:
        //    Serial.printf("SOL_C=");
        //    Serial.println(decoded_data);
            break;

            case 3:
        //    Serial.printf("LMP_FLSH=");
        //    Serial.println(decoded_data);
            break;

            case 6:
        //    Serial.printf("AUX_DRV=");
        //    Serial.println(decoded_data);
            break;
              
            case 8:

            // LMP_STB
            
            lamp_strobe = concatenateRegisters(decoded_data,aux_lamp);
//            Serial.println(lamp_strobe,BIN);
//            update_matrix(lamp_strobe, lamp_drive, lamp_matrix);
                       
            break;

            case 9:
            // AUX_LMP

            lamp_strobe = concatenateRegisters(lamp_strobe,decoded_data);
//            Serial.println(lamp_strobe,BIN);
//            update_matrix(lamp_strobe, lamp_drive, lamp_matrix);
                        
            case 10:

            // LMP_DRV

            lamp_drive=decoded_data;
            update_matrix(lamp_strobe, lamp_drive, lamp_matrix);
         
            break;

            }
        }
 
 check_index();


}
         

void setup1()
{

}

void loop1()

{
 //    ws2812fx.service();



#ifdef DEBUG_LAMP_STATE

// Print lamp state matrix
for(int i = 0; i < 10; i++) {
    for(int j = 0; j < 8; j++) {
        Serial.print(lamp_state[i][j]);
        Serial.print(" ");
    }
    Serial.println();
}

    Serial.println();

#endif

 }
