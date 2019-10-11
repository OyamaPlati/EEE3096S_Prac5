#ifndef PRAC5_H
#define PRAC5_H

//Includes
#include <wiringPi.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>

#include <wiringPiI2C.h>
#include <softPwm.h>
#include "BinClock.h"
#include "CurrentTime.h"

#include <mcp3004.h>

// Define buttons
#define CONTROL_BUTTON 0	// Set 0 (Wiring Pi) : Physical pin 11 : Start/Stop monitoring
#define DISMISS_BUTTON 2	// Set 2 (Wiring Pi) : Physical pin 13 : Dismiss alarm
#define RESET_BUTTON 4		// Set 4 (Wiring Pi): Physical pin 16: Reset system time
#define FREQUENCY_BUTTON 5	// Set 5 (Wiring Pi): Physical Pin 18: Changing interval reading

// Set up SPI channels
#define BASE 100                // Pin base for the chip
#define SPI_CHAN 0 		// Use Raspberry Pi SPI channel 0
#define SPI_SPEED 500000 	// Choose 32 kHz clock speed (Nyquist) wth 8/5 scaling factor = 409.6 kHz

// Define thresholds
#define LOWER_LIMIT 0.65	// Set lower limit to 0.65 V
#define UPPER_LIMIT 2.65	// Set upper limit to 2.65 V

// Function definitiions
int setup_gpio(void);
int main(void);

#endif

