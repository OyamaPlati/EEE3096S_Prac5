
#include "Prac5.h"

using namespace std;

// Define class variables
bool monitoring = true;		// Set false when stopped
bool dismissed = false; 	// If true, alarm should stop

int hours, mins, secs;
int RTC; 			// Holds the RTC instance
int HH,MM,SS;

unsigned char buffer[2][BUFFER_SIZE][2];
int buffer_location = 0;
bool bufferReading = 0;        // Using this to switch between column 0 and 1 - the first column
bool threadReady = false;      // Using this to finish writing the first column at the start of the song,
			       // before the column is played

long lastInterruptTime = 0;	// Define start of interrupt time


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
        printf("Interrupt Control triggered\n");
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
        printf("Interrupt DISMISS triggered\n");
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
        printf("Interrupt RESET triggered\n");
        // Write the logic here
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
        printf("Interrupt FREQUENCY triggered\n");
	// Write the logic here
    }
    lastInterruptTime = interruptTime;
}

/**
 * Thread that handles reading from SPI
 *
 * You must stop reading from SPI if not monitoring is true (the monitoring is dismissed)
 * When calling the function to read from SPI, take note of the last argument.
 * You don't need to use the returned value from the wiring pi SPI function
 * You need to use the buffer_location variable to check when you need to switch buffers.
 *
 */

void *monitorThread(void *threadargs){
    // If the thread isn't ready, don't do anything
    while(!threadReady)
        continue;

    // You need to only be monitoring if the dismissed flag is false
    while(!dismissed){
        // Code to suspend monitoring if dismissed
        while(!monitoring)
            continue;

        // Read the buffer out from SPI
        wiringPiSPIDataRW (SPI_CHAN, buffer[bufferReading][buffer_location], 2) ;

        // Do some maths to check if you need to toggle buffers
        buffer_location++;
        if(buffer_location >= BUFFER_SIZE) {
            buffer_location = 0;
            bufferReading = !bufferReading; // Switches column one it finishes one column
        }
    }

    pthread_exit(NULL);
}

/**
 * setup_gpio subroutine to set up gpio. Called once.
 *
 */

int setup_gpio(void){
     printf("SETTING UP\n");

    //Set up wiring Pi
    wiringPiSetup();

    //setting up the buttons
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
    printf("BUTTONS DONE\n");

    // Setting up the SPI interface
    // Use SPI channel 0 and Clock speed
    if (wiringPiSPISetup(SPI_CHAN, SPI_SPEED) == -1) {
        printf("Unable to setup SPI interface: %s\n", strerror(errno));
    }
    printf("SPI DONE\n");

    RTC = wiringPiI2CSetup(RTCAddr); // Set up the RTC
    printf("RTC DONE\n");

    printf("SETUP DONE\n");
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

    // Program loop

    

    // Join and exit the monitorthread
    pthread_join(thread_id, NULL);
    pthread_exit(NULL);


    return 0;
}
