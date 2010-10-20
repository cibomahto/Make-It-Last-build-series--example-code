/////////////////////////////////////////////////////////////////////////////
// Include files
// These lines allow us to use code routines (libraries) from other files,
// so that we don't have to write everything by ourselves.
/////////////////////////////////////////////////////////////////////////////

#include <p18lf25k22.h>	    // This file includes definitions for all of the
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

// Function that takes roughly 1 second to run
void waitASecond(void) {
    int i;
    int j;

    // It takes the microcontroller approximately one second to run through
    // this counting loop
    for (i = 0; i < 255; i++) {
        for (j = 0; j < 750; j++) {
        }
    }
}

// Function to delay for a specified number of seconds
void wait( int time ) {
    int i;

    // Call the waitASecond() function repeatedly until we have waited the
    // specified number of seconds
    for (i = 0; i < time; i++) {
        waitASecond();
    }
}

// Function to write a character string to the serial port, from:
// C:\MCC18\src\pmc_common\USART\u1puts.c
void puts1USART( char *data)
{
    do
    { // Transmit a byte
        while(!TXSTA1bits.TRMT);
        TXREG1 = *data; // Write the data byte to the USART2
    } while( *data++ );
}


// Map function, from:
// http://www.arduino.cc/en/Reference/Map
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// This function is called once, when the microcontroller is turned on.
void setup(void) {
  // Processor configuration

    // Configure the oscillator to run at 16 MHz
    OSCCONbits.IRCF = 111;

  // ADC configuration

    // Configure the internal voltage regulator to output 2.048V
    // This voltage is stable over varying battery voltage, and is used
    // as a reference when measuring the thermometer.
    VREFCON0bits.FVREN = 1;
    VREFCON0bits.FVRS = 0b10; // Positive reference = 2.048V

    // Set the ADC to use ground (0V) for the negative reference, and
    // the internal voltage regulator (2.048V) as the positive reference.
    ADCON1bits.NVCFG = 0;     // Negative reference = 0V
    ADCON1bits.PVCFG = 0b10;  // Positive reference = 2.048V

    // Set the speed that the ADC should capture data
    ADCON2bits.ADFM=1;        // Right Justified
    ADCON2bits.ACQT=0b111;    // Acquisition time
    ADCON2bits.ADCS=0b010;    // Fosc/32

    // Configure pin 13 (port C, pin 2) as an input
    TRISCbits.TRISC2 = 1;	  // Make the pin an input
    ANSELCbits.ANSC2 = 1;     // Specifically, an analog input

    // Tell the ADC to use pin 13 (input 14) as the input, and turn on the ADC
    ADCON0bits.CHS = 14;      // Select pin 13 as the input
    ADCON0bits.ADON = 1;      // Turn the ADC on


// Serial port configuration
    // Configure the serial port to run at 9600 baud
    // (see manual, page 275)
    BAUD1CONbits.BRG16 = 0;
    SPBRG1 = 25;
    TXSTA1bits.BRGH = 0; // Baud rate select

    // Enable the serial port
    TXSTA1bits.TXEN = 1; // Enable transmitter
    RCSTA1bits.SPEN = 1; // Enable receiver
}


// This function is called repeatedly
void loop(void) {
    unsigned int results = 0;
    unsigned int temperature;
    unsigned char i;
    char buffer[30];

    // Read the ADC port a bunch of times and average the results together,
    // so that we get a more stable output. Taking a single sample can result
    // in noisy data!
    for (i = 0; i < 64; i++) {
        // Signal the ADC to start conversion
        ADCON0bits.GO = 1;

        // And wait till it finishes
        while (ADCON0bits.GO == 1) {}

		// Then, add the result to the result count
        results += ((int)ADRESH << 8) + ADRESL;
    }

    // Finally, determine the average by dividing the total counts by the
    // number of samples that were taken.
    temperature = results/64;

    // Convert the ADC counts into a temperature
    // Be sure to fill in your own measurements for the first two values!
    // (freezing and boiling measurements, in counts)
    temperature = map(temperature, 255, 738, 0, 100);   // Celcius
//    temperature = map(temperature, 255, 738, 32, 212);  // Fahrenheit

    // Write the results out to a string
    sprintf(buffer,"%u\n\r", temperature);

    // Then send that string to the serial port
    puts1USART(buffer);

   // Wait for five seconds before checking the temperature again.
   wait(5);
}
