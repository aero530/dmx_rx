/*
 * Copyright (c) 2021 Jostein Løwer 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * Description: 
 * Starts a DMX Input on GPIO pin 0 and read channel 1-3 repeatedly
 */

#include <Arduino.h>
#include <Wire.h>
#include "DmxInput.h"

DmxInput dmxInput;

#define START_CHANNEL 1
#define NUM_CHANNELS 512
#define BUFFER_SIZE DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)

// Buffer for use with DMX library
volatile uint8_t buffer[BUFFER_SIZE];

// Buffer to share data between cores
volatile uint8_t sharedData[BUFFER_SIZE]; 

// Flag to signal new data is available (needs synchronization)
volatile bool newDataReady = false; 

void setup() {
    Serial.begin(115200);

    // Setup our DMX Input to read on GPIO 0
    dmxInput.begin(0, START_CHANNEL, NUM_CHANNELS);
    dmxInput.read_async(buffer);

    // Setup the onboard LED so that we can blink when we receives packets
    pinMode(LED_BUILTIN, OUTPUT);
    
    rp2040.fifo.begin(2);
}

void loop() {
    delay(30);

    if(millis() > 100+dmxInput.latest_packet_timestamp()) {
        Serial.println("no data!");
        return;
    }

    if (!newDataReady) { // Only fill if Core 1 has processed previous data
        for (int i = 0; i < BUFFER_SIZE; i++) {
            sharedData[i] = i % 256; 
        }
        Serial.println("Core 0: Filled data buffer."); 
        newDataReady = true; // Signal new data is ready for Core 1
        rp2040.fifo.push(1); // Push a dummy value to FIFO to wake up Core 1
    }

    // Blink the LED to indicate that a packet was received
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
}


byte core1Data[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)]; 
volatile bool i2cDataRequest = false;

void setup1() {
  Wire.setSDA(4); //GPIO4 = pin 6
  Wire.setSCL(5); //GPIO5 = pin 7
  Wire.begin(0x33);
  Wire.onRequest(req);
}


void loop1() {
    if (rp2040.fifo.available()) { // Check if FIFO has data (signal from Core 0)
        rp2040.fifo.pop(); // Pop the dummy value
        Serial.println("Core 1: Received signal from Core 0."); 

        // Process the data in the shared buffer
        for (int i = 0; i < BUFFER_SIZE; i++) {
            core1Data[i] = sharedData[i];
        }
        
        newDataReady = false; // Reset the flag, allowing Core 0 to fill again
        Serial.println("Core 1: Data transferred."); 
    }

    if (i2cDataRequest == true) {
        Serial.println("I2C Data request");
        Wire.write(core1Data, BUFFER_SIZE);
        i2cDataRequest = false;
    }
    
    delay(10);
}

// These are called in an **INTERRUPT CONTEXT** which means NO serial port
// access (i.e. Serial.print is illegal) and no memory allocations, etc.

// Called when the I2C slave is read from
void req() {
    i2cDataRequest = true; 
}