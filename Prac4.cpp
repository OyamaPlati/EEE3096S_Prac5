/*
 * Prac4.cpp
 *
 * Originall written by Stefan Schröder and Dillion Heald
 *
 * Adapted for EEE3096S 2019 by Keegan Crankshaw
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "Prac4.h"

using namespace std;
bool playing = true; // should be set false when paused
bool stopped = false; // If set to true, program should close
unsigned char buffer[2][BUFFER_SIZE][2];
int buffer_location = 0;
bool bufferReading = 0; //using this to switch between column 0 and 1 - the first column
bool threadReady = false; //using this to finish writing the first column at the start of the song, before the column is played
long lastInterruptTime = 0;


// Configure your interrupts here.
// Don't forget to use debouncing.

/*
 * Pause playbeck should stop playback and reading from the buffer when first pressed.
 * When pressed again, playback should continue.
 */
void play_pause_isr(void){
    // Debounce
    long interruptTime = millis();
    if (interruptTime - lastInterruptTime > 200){
        printf("Interrupt PLAY triggered\n");
        // Write your logic here
	playing = !playing; // Pause playback
    }
    lastInterruptTime = interruptTime;
}

/*
 * Stop playback could be considered as an "exit" as well
 */

void stop_isr(void){
    // Debounce
    long interruptTime = millis();
    if (interruptTime - lastInterruptTime > 200){
        printf("Interrupt STOP triggered\n");
        // Write your logic here
	stopped = !stopped; // Stop playback
	exit(0);
    }
    lastInterruptTime = interruptTime;
}

/*
 * Setup Function. Called once
 */
int setup_gpio(void){
    printf("SETTING UP\n");

    //Set up wiring Pi
    wiringPiSetup();

    //setting up the buttons
    pinMode(PLAY_BUTTON, INPUT); // Set-up the play button, pin 3 (Wiring Pi) : Physical Pin 15
    pullUpDnControl(PLAY_BUTTON, PUD_DOWN); // Set pin 3 (Wiring Pi) to be an input pin and set initial value to be pull
    pinMode(STOP_BUTTON, INPUT); // Set-up the stop button, 4 (Wiring Pi) : Physical Pin 16
    pullUpDnControl(STOP_BUTTON, PUD_DOWN); // Set pin 4 (Wiring Pi) to be an input pin and set initial value to be pull

    // Attach interrupts to Buttons : Set-up event on pin 3 and 4 for rising edge
    if (wiringPiISR(PLAY_BUTTON, INT_EDGE_RISING, &play_pause_isr) < 0) {
        printf("Unable to setup ISR: %s\n", strerror(errno));
    }
    else if (wiringPiISR(STOP_BUTTON, INT_EDGE_RISING, &stop_isr) < 0) {
        printf("Unable to setup ISR: %s\n", strerror(errno));
    }
    printf("BUTTONS DONE\n");


    //setting up the SPI interface
    // Use SPI channel 0 and Clock speed 409.6 kHz
    if (wiringPiSPISetup(SPI_CHAN, SPI_SPEED) == -1) {
        printf("Unable to setup SPI: %s\n", strerror(errno));
    }
    printf("SPI DONE\n");

    printf("SETUP DONE\n");
    return 0;
}

/*
 * Thread that handles writing to SPI
 *
 * You must pause writing to SPI if not playing is true (the player is paused)
 * When calling the function to write to SPI, take note of the last argument.
 * You don't need to use the returned value from the wiring pi SPI function
 * You need to use the buffer_location variable to check when you need to switch buffers
 */
void *playThread(void *threadargs){
    // If the thread isn't ready, don't do anything
    while(!threadReady)
        continue;

    //You need to only be playing if the stopped flag is false
    while(!stopped){
        //Code to suspend playing if paused
	//TODO
        while (!playing)
	    continue;

	//Write the buffer out to SPI
        //TODO
        wiringPiSPIDataRW (SPI_CHAN, buffer[bufferReading][buffer_location], 2);


        //Do some maths to check if you need to toggle buffers
        buffer_location++;
        if(buffer_location >= BUFFER_SIZE) {
            buffer_location = 0;
            bufferReading = !bufferReading; // switches column one it finishes one column
        }
    }

    pthread_exit(NULL);
}

int main(){
    // Call the setup GPIO function
	if(setup_gpio()==-1){
        return 0;
    }

    /* Initialize thread with parameters
     * Set the play thread to have a 99 priority
     * Read https://docs.oracle.com/cd/E19455-01/806-5257/attrib-16/index.html
     */

    //Write your logic here
    pthread_attr_t tattr;
    pthread_t thread_id;
    int newprio = 99;
    sched_param param;

    pthread_attr_init (&tattr); // initialized with default attributes
    pthread_attr_getschedparam (&tattr, &param); // safe to get existing scheduling param
    param.sched_priority = newprio; // set the priority, others are unchanged
    pthread_attr_setschedparam (&tattr, &param); // setting the new scheduling param
    pthread_create(&thread_id, &tattr, playThread, (void *)1); // with new priority specified

    /*
     * Read from the file, character by character
     * You need to perform two operations for each character read from the file
     * You will require bit shifting
     *
     * buffer[bufferWriting][counter][0] needs to be set with the control bits
     * as well as the first few bits of audio
     *
     * buffer[bufferWriting][counter][1] needs to be set with the last audio bits
     *
     * Don't forget to check if you have pause set or not when writing to the buffer
     *
     */

    // Open the file
    char ch;
    FILE *filePointer;
    printf("%s\n", FILENAME);
    filePointer = fopen(FILENAME, "r"); // read mode

    if (filePointer == NULL) {
        perror("Error while opening the file.\n");
        exit(EXIT_FAILURE);
    }

    int counter = 0;
    int bufferWriting = 0;

    // Have a loop to read from the file
    while((ch = fgetc(filePointer)) != EOF){
        while(threadReady && bufferWriting==bufferReading && counter==0){
            // waits in here after it has written to a side, and the thread is still reading from the other side
            continue;
        }

        // Bit shifting is used so that the 8 bits you read from the file can be correctly distributed
        // among the 16 bits that you have to write over SPI.
        // The way the bits are laid out, and what all the flags mean, can be found in the datasheet.

        // Set config bits for first 8 bit packet and OR with upper bits

        char value = fgetc(filePointer);
        //bitset<8> y(value);
        //cout << "value " << y << " \n";

        char upper = ((value >> 6) & 0xff) | 0b0 << 7 | 0b0 << 6 | 0b1 << 5 | 0b1 << 4;
        //bitset<8> y_(upper);
        //cout << "upper " << y_ << " \n";

        char lower = value << 2 & 0b11111100;
        //bitset<8> g(lower);
        //cout << "2nd byte " << g << " \n";


	buffer[bufferWriting][counter][0] = upper; //TODO
        // Set next 8 bit packet
        buffer[bufferWriting][counter][1] = lower; //TODO


        counter++;
        if(counter >= BUFFER_SIZE+1){
            if(!threadReady){
                threadReady = true;
            }

            counter = 0;
            bufferWriting = (bufferWriting+1)%2;
        }
    }

    // Close the file
    fclose(filePointer);
    printf("Complete reading");

    //Join and exit the playthread
    pthread_join(thread_id, NULL);
    pthread_exit(NULL);

    return 0;
}


