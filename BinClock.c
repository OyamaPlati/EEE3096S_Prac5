/*
 * BinClock.c
 * Jarrod Olivier
 * Modified for EEE3095S/3096S by Keegan Crankshaw
 * August 2019
 *
 * <CHNYON001> <PLTOYA001>
 * Date 21 August 2019
*/

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h> //For printf functions
#include <stdlib.h> // For system functions
#include <softPwm.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include "BinClock.h"
#include "CurrentTime.h"

//Global variables
int hours, mins, secs;
long lastInterruptTime = 0; //Used for button debounce
int RTC; //Holds the RTC instance
int HH,MM,SS;

void initGPIO(void){
	/*
	 * Sets GPIO using wiringPi pins. see pinout.xyz for specific wiringPi pins
	 * You can also use "gpio readall" in the command line to get the pins
	 * Note: wiringPi does not use GPIO or board pin numbers (unless specifically set to that mode)
	 */
	printf("Setting up\n");
	wiringPiSetup(); //This is the default mode. If you want to change pinouts, be aware
	
	RTC = wiringPiI2CSetup(RTCAddr); //Set up the RTC
	
	//Set up the LEDS
	for(int i=0; i < sizeof(LEDS)/sizeof(LEDS[0]); i++){
	    pinMode(LEDS[i], OUTPUT);
	}
	
	//Set Up the Seconds LED (Wiring Pin 1) for PWM
	if(softPwmCreate(SECS,0,60) != 0){
		printf("Unable to setup PWM : %s\n", strerror(errno));
	}

	pinMode(SECS, PWM_OUTPUT);
	pwmSetRange(60);

	printf("LEDS done\n");
	
	//Set up the Buttons
	for(int j=0; j < sizeof(BTNS)/sizeof(BTNS[0]); j++){
		pinMode(BTNS[j], INPUT);
		pullUpDnControl(BTNS[j], PUD_UP);
	}
	
	//Attach interrupts to Buttons
	if (wiringPiISR(BTNS[0], INT_EDGE_RISING, &hourInc) < 0) {
		printf("Unable to setup ISR: %s\n", strerror(errno));
	}
	else if (wiringPiISR(BTNS[1], INT_EDGE_RISING, &minInc) < 0) {
                printf("Unable to setup ISR: %s\n", strerror(errno));
        }

	printf("BTNS done\n");
	printf("Setup done\n");
}

/*
 * The main function
 * This function is called, and calls all relevant functions we've written
 */
int main(void){
	initGPIO();

	//Set random time (3:04PM)
	wiringPiI2CWriteReg8(RTC, HOUR, 0x13+TIMEZONE);
	wiringPiI2CWriteReg8(RTC, MIN, 0x4);
	wiringPiI2CWriteReg8(RTC, SEC, 0b10000000);

	// Repeat this until we shut down
	for (;;){
		//Fetch the time from the RTC
		HH = wiringPiI2CReadReg8(RTC, HOUR);
		MM = wiringPiI2CReadReg8(RTC, MIN);
		SS = wiringPiI2CReadReg8(RTC, SEC);

		hours = hexCompensation(HH);
		mins = hexCompensation(MM);
		secs = hexCompensation(SS & 0b01111111);

		//Function calls to toggle LEDs
		hours = hFormat(hours);
		lightHours(hours);

		mins = mFormat(mins);
		lightMins(mins);

		secPWM(secs);


		// Print out the time we have stored on our RTC
		printf("The current time is: %d:%d:%d\n", hours, mins, secs);

		//using a delay to make our program "less CPU hungry"
		delay(1000); //milliseconds
	}

	// cleanup the environment. set each pin to low
    	// and set the mode to INPUT. These steps make sure
    	// the equipment is safe to manipulate and prevents
    	// possible short and equipment damage from energized pin.
	
	digitalWrite(LEDS[0], LOW);
	for (int i = 0; i < sizeof(LEDS)/sizeof(LEDS[0]); i++) { pinMode(LEDS[i], INPUT); }
    	pinMode(SECS, INPUT);
    	digitalWrite (SECS, LOW);
	
	return 0;
}

/*
 * Change the hour format to 12 hours
 */
int hFormat(int hours){
	/*formats to 12h*/
	if (hours >= 24){
		hours = 1;
	}
	else if (hours > 12){
		hours -= 12;
	}
	return (int)hours;
}

int mFormat(int mins){

        if (mins >= 60){
		HH = hexCompensation(HH); // increment by 1
		HH++;
		HH = decCompensation(HH);
		mins = 0;
        }

        return (int)mins;
}

/*
 * Turns on corresponding LED's for hours
 */
void lightHours(int units){
	// Write your logic to light up the hour LEDs here
	if (units >= 8) {digitalWrite(0, HIGH); units -= 8;}
	else {digitalWrite(0, LOW);}
	if (units >= 4) {digitalWrite(2, HIGH); units -= 4;}
	else {digitalWrite(2, LOW);}
	if (units >= 2) {digitalWrite(3, HIGH); units -= 2;}
	else {digitalWrite(3, LOW);}
	if (units >= 0) {digitalWrite(25, HIGH); units -= 0;}
	else {digitalWrite(25, LOW);}
}

/*
 * Turn on the Minute LEDs
 */
void lightMins(int units){
	//Write your logic to light up the minute LEDs here
	if (units >= 32) {digitalWrite(7, HIGH); units -= 32;}
	else {digitalWrite(7, LOW);}
	if (units >= 16) {digitalWrite(22, HIGH); units -= 16;}
	else {digitalWrite(22, LOW);}
	if (units >= 8) {digitalWrite(21, HIGH); units -= 8;}
	else {digitalWrite(21, LOW);}
        if (units >= 4) {digitalWrite(27, HIGH); units -= 4;}
	else {digitalWrite(27, LOW);}
	if (units >= 2) {digitalWrite(4, HIGH); units -= 2;}
	else {digitalWrite(4, LOW);}
	if (units >= 0) {digitalWrite(6, HIGH); units -= 0;}
	else {digitalWrite(6, LOW);}
}

/*
 * PWM on the Seconds LED
 * The LED should have 60 brightness levels
 * The LED should be "off" at 0 seconds, and fully bright at 59 seconds
 */
void secPWM(int units){
	// Write your logic here
	softPwmCreate(SECS, 0, 59);
	softPwmWrite(SECS, units);
}

/*
 * hexCompensation
 * This function may not be necessary if you use bit-shifting rather than decimal checking for writing out time values
 */
int hexCompensation(int units){
	/*Convert HEX or BCD value to DEC where 0x45 == 0d45
	  This was created as the lighXXX functions which determine what GPIO pin to set HIGH/LOW
	  perform operations which work in base10 and not base16 (incorrect logic)
	*/
	int unitsU = units%0x10;

	if (units >= 0x50){
		units = 50 + unitsU;
	}
	else if (units >= 0x40){
		units = 40 + unitsU;
	}
	else if (units >= 0x30){
		units = 30 + unitsU;
	}
	else if (units >= 0x20){
		units = 20 + unitsU;
	}
	else if (units >= 0x10){
		units = 10 + unitsU;
	}
	return units;
}


/*
 * decCompensation
 * This function "undoes" hexCompensation in order to write the correct base 16 value through I2C
 */
int decCompensation(int units){
	int unitsU = units%10;

	if (units >= 50){
		units = 0x50 + unitsU;
	}
	else if (units >= 40){
		units = 0x40 + unitsU;
	}
	else if (units >= 30){
		units = 0x30 + unitsU;
	}
	else if (units >= 20){
		units = 0x20 + unitsU;
	}
	else if (units >= 10){
		units = 0x10 + unitsU;
	}
	return units;
}


/*
 * hourInc
 * Fetch the hour value off the RTC, increase it by 1, and write back
 * Be sure to cater for there only being 23 hours in a day
 * Software Debouncing should be used
 */
void hourInc(void){
	//Debounce
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime > 200){
		printf("Interrupt 1 triggered, %d\n", hours);
		//Fetch RTC Time
		HH = wiringPiI2CReadReg8(RTC, HOUR);
		//Increase hours by 1, ensuring not to overflow
		HH = hexCompensation(HH);
		HH++;
                HH = decCompensation(HH);
		//Write hours back to the RTC
                wiringPiI2CWriteReg8(RTC, HOUR, HH);
	}
	lastInterruptTime = interruptTime;
}

/*
 * minInc
 * Fetch the minute value off the RTC, increase it by 1, and write back
 * Be sure to cater for there only being 60 minutes in an hour
 * Software Debouncing should be used
 */
void minInc(void){
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>200){
		printf("Interrupt 2 triggered, %d\n", mins);
		MM = wiringPiI2CReadReg8(RTC, MIN);
                //Increase hours by 1, ensuring not to overflow
                MM = hexCompensation(MM);
		MM++;
                MM = decCompensation(MM);
                //Write hours back to the RTC
                wiringPiI2CWriteReg8(RTC, MIN, MM);

	}
	lastInterruptTime = interruptTime;
}

//This interrupt will fetch current time from another script and write it to the clock registers
//This functions will toggle a flag that is checked in main
void toggleTime(void){
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>200){
		HH = getHours();
		MM = getMins();
		SS = getSecs();

		HH = hFormat(HH);
		HH = decCompensation(HH);
		wiringPiI2CWriteReg8(RTC, HOUR, HH);

		MM = decCompensation(MM);
		wiringPiI2CWriteReg8(RTC, MIN, MM);

		SS = decCompensation(SS);
		wiringPiI2CWriteReg8(RTC, SEC, 0b10000000+SS);

	}
	lastInterruptTime = interruptTime;
}
