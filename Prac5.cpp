
#include "Prac5.h"
#include "CurrentTime.c"

using namespace std;

// Define class variables
bool monitoring = true;	        // If true, start  monitoring
bool dismissed = false; 	// If true, alarm should stop
bool reset = true;		// If true, clear console and system timer

int hours, mins, secs;
int RTC; 			// Holds the RTC instance

long lastInterruptTime = 0;	// Define start of interrupt time

int tic = 1000;			// Define rate of monitoring. Default 1 second
int choice = 0;		        // Define frequency (0 = 1s, 1 = 2s and 2 = 5s)

int lastAlarmHour = 0;
int lastAlarmMin = 0;
int lastAlarmSec = 0;

int tempHour = 0;
int tempMin = 0;
int tempSec = 0;

// Configure interrupts here
// Use debouncing

/**
 * start_stop_isr subroutine to start monitoring once device is booted and
 * stop monitoring and printing out to console once stopped.
 *
 */
void start_stop_isr(void){
    long interruptTime = millis();
        if (interruptTime - lastInterruptTime > 200){
        monitoring = !monitoring;
    }
    lastInterruptTime = interruptTime;
}

/**
 * dismiss_isr subroutine to dismiss the alarm
 *
 */
void dismiss_isr(void){
    long interruptTime = millis();
    if (interruptTime - lastInterruptTime > 200){
        dismissed = !dismissed;
    }
    lastInterruptTime = interruptTime;
}

/**
 * reset_isr subroutine to clear console and reset system time
 *
 */
void reset_isr(void) {
    long interruptTime = millis();
    if (interruptTime - lastInterruptTime > 200){
	// Set start time (0:00)
        wiringPiI2CWriteReg8(RTC, HOUR, 0x0);
        wiringPiI2CWriteReg8(RTC, MIN, 0x0);
        wiringPiI2CWriteReg8(RTC, SEC, 0b10000000);
	system("clear");
	printf("%-10s%-10s%-10s%-10s%-10s%-10s%-10s\n", "RTC Time", "Sys Timer", "Humidity", "Temp", "Light", "DAC Vout", "Alarm");
    }
    lastInterruptTime = interruptTime;
}

/**
 * frequency_isr subroutine to change rate of monitoring on range (1, 2 and 5 seconds).
 *
 */
void frequency_isr(void) {
    long interruptTime = millis();
    if (interruptTime - lastInterruptTime > 200){
        switch(choice) {
	    case 0:
		tic = 1000;
		break;
	    case 1:
		tic = 2000;
		break;
	    case 2:
		tic = 5000;
		break;
	}
	choice = (choice + 1) % 3;
    }
    lastInterruptTime = interruptTime;
}

/**
 * monitorThread that handles reading from ADC
 *
 * You must stop reading from ADC if not monitoring is true (the monitoring is dismissed)
 * When calling the function to read from ADC, take note of the last argument.
 * You don't need to use the returned value from the wiring pi SPI function
 * You need to use the buffer_location variable to check when you need to switch buffers.
 *
 */
void *monitorThread(void *threadargs){
    for(;;){

	while (!monitoring) { 
		
		
        	continue; 
	
	}

        // Read from out from ADC
	// Fetch the time from the RTC
	HH = wiringPiI2CReadReg8(RTC, HOUR);
	MM = wiringPiI2CReadReg8(RTC, MIN);
	SS = wiringPiI2CReadReg8(RTC, SEC);

	hours = hexCompensation(HH);
	mins = hexCompensation(MM);
	secs = hexCompensation(SS & 0b01111111);

	// Function calls
	hours = hFormat(hours);
        mins = mFormat(mins);

        string hourStr = to_string(hours);
        string minStr = to_string(mins);
	string secStr = to_string(secs);
        string str = string(hourStr + ":" + minStr + ":" + secStr);

	string currentHour = to_string(getHours());
	string currentMin = to_string(getMins());
	string currentSec = to_string(getSecs());
	string curStr = string(currentHour + ":" + currentMin + ":" + currentSec);

	// Covert Humidity
	int humValue = analogRead(BASE + 7);
	float humidity = (humValue * 3.3) / 1024.0;

	// Convert Temperature
	int tempValue =  analogRead(BASE + 2);
	float volts = (tempValue * 3.3) / 1024.0;
	float temperature = (volts - (500.0 / 1000)) / (10.0 / 1000);

	// Calculate DAC Vout
	int lightValue = analogRead(BASE + 0);
	float dacVout = (lightValue / 1024.0) * humidity;
	int dacValue = (int) (1024 * dacVout) / 3.3;

	if ((dacVout < LOWER_LIMIT) || (dacVout > UPPER_LIMIT)) {
	   if ((interval(hours, mins, secs,
		lastAlarmHour, lastAlarmMin, lastAlarmSec) == true) || (!dismissed)){
		secPWM(dacValue);
		printf("%-10s%-10s%-10.2f%-10.2f%-10d%-10.2f%-10s\n", curStr.c_str(), str.c_str(), humidity, temperature, lightValue, dacVout, "*");
		lastAlarmHour = hours;
		lastAlarmSec = secs;
		lastAlarmMin = mins;
	   }
	   else {
		secPWM(0);
	    	printf("%-10s%-10s%-10.2f%-10.2f%-10d%-10.2f%-10s\n", curStr.c_str(), str.c_str(), humidity, temperature, lightValue, dacVout, " ");
	   }
	}
	else {
	    secPWM(0);
	    printf("%-10s%-10s%-10.2f%-10.2f%-10d%-10.2f%-10s\n", curStr.c_str(), str.c_str(), humidity, temperature, lightValue, dacVout, " ");
	}

	// Using a delay to make our program "less CPU hungry"
	delay(tic);
    }

    pthread_exit(NULL);
}

/**
 * secPWM subroutine
 * PWM on the Alarm LED
 * The LED should have 1024 brightness levels
 * The LED should be "off" at 0 Volts, and fully bright at 3.3 Volts
 *
 */
void secPWM(int units){
    softPwmCreate(SECS, 0, 1023);
    softPwmWrite(SECS, units);
}

/**
 * makeSound subroutine alarm sounded if last alarm was sounded less than 3 minutes prior
 *
 */
bool interval (int hour1, int minute1, int second1,
		int hour2, int minute2, int second2) {
    int diff_hour, diff_minute, diff_second;

    if(second2 > second1) {
      minute1--;
      second1 += 60;
   }

   diff_second = second1 - second2;

   if(minute2 > minute1) {
      hour1--;
      minute1 += 60;
   }

   diff_minute = minute1 - minute2;
   diff_hour = hour1 - hour2;

   if ((diff_hour >= 0) && (diff_second >= 0) && (diff_minute >= 3)) {

	return true;
   }
   else {

	return false;
   }
}

/**
 * setup_gpio subroutine to set up gpio. Called once.
 *
 */
int setup_gpio(void){
     //printf("SETTING UP\n");

    // Set up wiring Pi
    wiringPiSetup();

    // Setting up the buttons
    pinMode(CONTROL_BUTTON, INPUT); // Set-up the control button, pin 0 (Wiring Pi) : Physical Pin 11
    pullUpDnControl(CONTROL_BUTTON, PUD_DOWN); // Set pin 0 (Wiring Pi) to be an input pin and set initial value to be pull down
    pinMode(DISMISS_BUTTON, INPUT); // Set-up the dismiss button, pin 2 (Wiring Pi) : Physical Pin 13
    pullUpDnControl(DISMISS_BUTTON, PUD_DOWN); // Set pin 2 (Wiring Pi) to be an input pin and set initial value to be pull down
    pinMode(RESET_BUTTON, INPUT); // Set-up the reset button, pin 4 (Wiring Pi) : Physical Pin 16
    pullUpDnControl(RESET_BUTTON, PUD_DOWN); // Set pin 4 (Wiring Pi) to be an input pin and set initial value to be pull down
    pinMode(FREQUENCY_BUTTON, INPUT); // Set-up the frequency button, pin 5 (Wiring Pi) : Physical Pin 18
    pullUpDnControl(FREQUENCY_BUTTON, PUD_DOWN); // Set pin 5 (Wiring Pi) to be an input pin and set initial value to be pull down

    // Attach interrupts to Buttons : Set-up event on pin 0, 2, 4 and 5 for rising edge
    if (wiringPiISR(CONTROL_BUTTON, INT_EDGE_RISING, &start_stop_isr) < 0) {
        printf("Unable to setup control ISR: %s\n", strerror(errno));
    }
    else if (wiringPiISR(DISMISS_BUTTON, INT_EDGE_RISING, &dismiss_isr) < 0) {
        printf("Unable to setup dismiss ISR: %s\n", strerror(errno));
    }
    else if (wiringPiISR(RESET_BUTTON, INT_EDGE_RISING, &reset_isr) < 0) {
        printf("Unable to setup reset ISR: %s\n", strerror(errno));
    }
    else if (wiringPiISR(FREQUENCY_BUTTON, INT_EDGE_RISING, &frequency_isr) < 0) {
        printf("Unable to setup frequency ISR: %s\n", strerror(errno));
    }
    //printf("BUTTONS DONE\n");

    // Setting up the SPI interface
    // Use SPI channel 0 for MCP30008 and Clock speed
    if (wiringPiSPISetup(SPI_CHAN, SPI_SPEED) == -1) {
        printf("Unable to setup SPI interface: %s\n", strerror(errno));
    }
    //printf("SPI DONE\n");

    // Setting up the mcp3008 chip
    mcp3004Setup(BASE, SPI_CHAN);
    //printf("MCP3008 DONE\n");

    //Set Up the Seconds LED (Wiring Pin 1) for PWM
    if(softPwmCreate(SECS, 0, 1024) != 0){
	printf("Unable to setup PWM: %s\n", strerror(errno));
    }
    pinMode(SECS, PWM_OUTPUT);

    pwmSetRange(60);
    //printf("PWM DONE\n");

    // Setting up the RTC
    RTC = wiringPiI2CSetup(RTCAddr);
    //printf("RTC DONE\n");

    //printf("SETUP DONE\n");
    return 0;
}

int main(){
    // Call the setup GPIO function
    if(setup_gpio() == -1){
        return 0;
    }

    // Initialize thread with parameters
    // Set the play thread to have a 99 priority
    pthread_attr_t tattr;
    pthread_t thread_id;
    int newprio = 99;
    sched_param param;

    pthread_attr_init (&tattr); 		 // Initialized with default attributes
    pthread_attr_getschedparam (&tattr, &param); // Safe to get existing scheduling param
    param.sched_priority = newprio; 		 // Set the priority; others are unchanged
    pthread_attr_setschedparam (&tattr, &param); // Setting the new scheduling param
    pthread_create(&thread_id, &tattr, monitorThread, (void *)1); // with new priority specified

    // Set start time (0:00)
    wiringPiI2CWriteReg8(RTC, HOUR, 0x0);
    wiringPiI2CWriteReg8(RTC, MIN, 0x0);
    wiringPiI2CWriteReg8(RTC, SEC, 0b10000000);

    // Program loop
    // Repeat this until we shut down
    printf("%-10s%-10s%-10s%-10s%-10s%-10s%-10s\n", "RTC Time", "Sys Timer", "Humidity", "Temp", "Light", "DAC Vout", "Alarm");

    // Join and exit the monitorthread
    pthread_join(thread_id, NULL);
    pthread_exit(NULL);

    return 0;
}

/**
 * Change the hour format to 12 hours
 *
 */
int hFormat(int hours){
    // Formats to 12h
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
        HH = hexCompensation(HH); // Increment by 1
        HH++;
        HH = decCompensation(HH);
        mins = 0;
    }
    return (int)mins;
}

/**
 * hexCompensation
 * This function may not be necessary if you use bit-shifting rather than decimal checking
 * 	for writing out time values
 *
 */
int hexCompensation(int units){
	// Convert HEX or BCD value to DEC where 0x45 == 0d45
	// This was created as the lighXXX functions which determine what GPIO pin to set HIGH/LOW
	// perform operations which work in base10 and not base16 (incorrect logic)
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

/**
 * decCompensation
 * This function "undoes" hexCompensation in order to write the correct base 16 value through I2C
 *
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
