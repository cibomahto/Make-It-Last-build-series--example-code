/////////////////////////////////////////////////////////////////////////////
// Include files
// These lines allow us to use code routines (libraries) from other files,
// so that we don't have to write everything by ourselves.
/////////////////////////////////////////////////////////////////////////////

#include <p18lf25k22.h>		// This file includes definitions for all of the
                            // registers on our chip
#include <stdio.h>          // For the sprintf() function


/////////////////////////////////////////////////////////////////////////////
// Pragma statements
// These lines tell the microcontroller what configuration to use when it
// when it turns on. The most important part for now is to tell the
// microcontroller what to use for a clock input.
/////////////////////////////////////////////////////////////////////////////

// Use the internal oscillator as a clock
#pragma config FOSC = INTIO67

// Configure it to run at 16 MHz
#pragma config PLLCFG = OFF

// Make sure that the internal watchdog timer is off. The watchdog is a
// a special feature of the processor, that runs separately from the main
// program and can be used to clea
#pragma config WDTEN = OFF


/////////////////////////////////////////////////////////////////////////////
// Function declarations
// Declare any user functions that you want to use here
/////////////////////////////////////////////////////////////////////////////
void main (void);
void setup(void);
void loop(void);


/////////////////////////////////////////////////////////////////////////////
// Function definitions
// Define any user functions that you want to use here
/////////////////////////////////////////////////////////////////////////////

// The main() function is where the program 'starts' when the microcontroller
// is turned on. For this project, we will use it to call setup() and loop()
// functions, then use those similar to an Arduino sketch.
void main (void)
{
    // Call the setup function once to put everything in order
    setup();

    // Then call the loop() function over and over
    while (1)
    {
        loop();
    }
}

// This function is called once, when the microcontroller is turned on.
void setup(void) {
    // Configure the oscillator to use a 16 MHz input.
    OSCCONbits.IRCF = 111;

    // Set all of the pins on Port A to be outputs,
    // Cleared bits are outputs.
    TRISA = 0;
    PORTA = 0x0;
}

// This function is called repeatedly
void loop(void) {
    // Make a variable for the display routine
    unsigned int count;

    // Turn the LED on
    PORTAbits.RA0 = 1;

    // Create a short delay
    for( count = 0; count < 64000; count++) {}

    // Turn the LED off
    PORTAbits.RA0 = 0;

    // Create a short delay
    for( count = 0; count < 64000; count++) {}
}
