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

#define NUMSAMPLES 50

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

// Wait for roughly 1 second (measured by hand)
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

// Wait a certain number of seconds
void wait( int time ) {
    int i;

    // Call the waitASecond() function repeatedly until we have waited the
    // specified number of seconds
    for (i = 0; i < time; i++) {
        waitASecond();
    }
}


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
    // Configure the oscillator to run at 16 MHz
    OSCCONbits.IRCF = 111;

    // Configure the internal voltage regulator to output 2.048V
    // This voltage is stable over varying battery voltage, and is used
    // as a reference when measuring the thermometer
    VREFCON0bits.FVREN = 1;
    VREFCON0bits.FVRS = 0b10; // Positive reference = 2.048V


    // Configure the ADC to use ground (0V) for the negative reference, and
    // the internal voltage regulator (2.048V) as the positive reference.
    ADCON1bits.NVCFG = 0; // Negative reference = 0V
    ADCON1bits.PVCFG = 0b10; // Positive reference = 2.048V


    // Conversion start & polling for completion
    // are included.
    ADCON2bits.ADFM=1; // Right Justified
    ADCON2bits.ACQT=0b111; // Acquisition time
    ADCON2bits.ADCS=0b010; // Fosc/32

    // Set port C, pin 2 as an input
    TRISCbits.TRISC2 = 1;
    ANSELCbits.ANSC2 = 1;


    // Select input 14 and turn on the ADC
    ADCON0bits.CHS = 14;
    ADCON0bits.ADON = 1;

// Turn on the EUSART
    TXSTA1 = 0; // Reset USART registers to POR state
    RCSTA1 = 0;

    BAUD1CONbits.BRG16 = 0;

    TXSTA1bits.BRGH = 0; // Baud rate select
    SPBRG1 = 25; // Assume 16MHz clock and 9600 baud serial

    PIE1bits.RC1IE = 0; // Disable serial interrupts
    PIE1bits.TX1IE = 0;

    TXSTA1bits.TXEN = 1; // Enable transmitter
    RCSTA1bits.SPEN = 1; // Enable receiver

    // Wait until the ADC is set up
    while (ADCON0bits.GO == 1) {}
}

// This function is called repeatedly
void loop(void) {
    unsigned int results = 0;
    unsigned int temperature;
    unsigned char i;
    char buffer[30];

    // read the ADC port a number of times
    for (i = 0; i < NUMSAMPLES; i++) {
        // Read the ADC
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO == 1) {}

        results += ((int)ADRESH << 8) + ADRESL;
    }
    temperature = results/NUMSAMPLES;

    // Convert the ADC counts into Celcius
    // Be sure to fill in your own measurements for the first two values!
    // (freezing and boiling measurements, in counts)
    temperature = map(temperature, 255, 738, 0, 100);

    // Or use this one for Fahrenheit
//    temperature = map(temperature, 255, 738, 32, 212);


    // then average the results to get a final reading
    sprintf(buffer,"%u\n\r", temperature);

    // Copy it out into a buffer and send to serial
    puts1USART(buffer);

   // now wait for five seconds
   wait(5);
}
