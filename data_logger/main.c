/////////////////////////////////////////////////////////////////////////////
// Include files
// These lines allow us to use code routines (libraries) from other files,
// so that we don't have to write everything by ourselves.
/////////////////////////////////////////////////////////////////////////////

#include <p18lf25k22.h>	    // This file includes definitions for all of the
                            // registers on our chip
#include <stdio.h>          // For the sprintf() function

#define __18LF25K80
#include <i2c.h>

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
void main ( void )
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
void waitASecond( void )
{
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
void wait( int time )
{
    int i;

    // Call the waitASecond() function repeatedly until we have waited the
    // specified number of seconds
    for (i = 0; i < time; i++) {
        waitASecond();
    }
}

// Function to write a character string to the serial port, from:
// C:\MCC18\src\pmc_common\USART\u1puts.c
void puts1USART( char *data )
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


// Function to write a single byte to a device on the i2c bus
void i2cWriteByte( char byte )
{
    // Send the first byte of address, and wait for ack
    SSPBUF = byte;

    // Wait for idle
    while ( ( SSPCON2 & 0x1F ) || ( SSPSTATbits.R_W ) )
        continue;
}


// Function to store a single byte of data in the EEPROM
void writeEEPROM( unsigned int address, unsigned char data )
{
    int i, j;

    // Enable the I2C port
    SSPCON2bits.SEN = 1;
    while (SSPCON2bits.SEN) {}

    // Send the EEPROM at address 7 a message that we are going to write
    // some data to it.
    i2cWriteByte(0b10101110);
    // Send over the data address we want to store the data at
    i2cWriteByte((address >> 8) & 0xFF);
    i2cWriteByte(address & 0xFF);

    // Then write the actual data to the device
    i2cWriteByte(data);

    // and close the i2c connection
    SSPCON2bits.PEN = 1;

    // Now, wait for the EEPROM to save it's data
    for (i = 0; i < 20; i++) {
        for (j = 0; j < 750; j++) {
        }
    }
}


// Function to read a single byte of data from the EEPROM
unsigned char readEEPROM( unsigned int address )
{
    unsigned char data = 0;

    // Enable the I2C port
    SSPCON2bits.SEN = 1;
    while (SSPCON2bits.SEN) {}

    // Send the EEPROM at address 7 a message that we are going to write
    // some data to it.
    i2cWriteByte(0b10101110);
    // Send over the data address we want to read from.
    i2cWriteByte((address >> 8) & 0xFF);
    i2cWriteByte(address & 0xFF);

    //  Now, reset communications and read the data in.
    SSPCON2bits.SEN = 1;
    while (SSPCON2bits.SEN) {}

    // Send the EEPROM at address 7 a message that we are going to read
    // some data from it.
    i2cWriteByte(0b10101111);

    // Read in the byte of data
    SSPCON2bits.RCEN = 1;
    while ( !SSPSTATbits.BF );
    data = SSPBUF;

    // and close the i2c connection
    SSPCON2bits.PEN = 1;

    return data;
}


unsigned int readTemperature()
{
    unsigned int results = 0;
    unsigned int temperature;
    int i;

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
//    temperature = map(temperature, 255, 738, 0, 100);   // Celcius
    temperature = map(temperature, 255, 738, 32, 212);  // Fahrenheit

    return temperature;
}


// This function is called once, when the microcontroller is turned on.
void setup( void ) {
  // Processor configuration

    // Configure the oscillator to run at 16 MHz
    OSCCONbits.IRCF = 111;

    TRISAbits.TRISA0 = 0;     // Make the LED pin an output
    PORTAbits.RA0 = 0;        // but turn it off to disable the LED

  // ADC configuration
    // Configure pin 13 (port C, pin 2) as an input
    TRISCbits.TRISC2 = 1;	  // Make the pin an input
    ANSELCbits.ANSC2 = 1;     // Specifically, an analog input

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

    // Tell the ADC to use pin 13 (input 14) as the input, and turn on the ADC
    ADCON0bits.CHS = 14;      // Select pin 13 as the input
    ADCON0bits.ADON = 1;      // Turn the ADC on


  // Serial port configuration
    TRISCbits.TRISC6 = 0;	  // Make TX pin an output
    TRISCbits.TRISC7 = 1;	  // and RX pin an input
    ANSELCbits.ANSC7 = 0;     // Specifically, an analog input

    // Configure the serial port to run at 9600 baud
    // (see manual, page 275)
    BAUD1CONbits.BRG16 = 0;
    SPBRG1 = 25;
    TXSTA1bits.BRGH = 0;      // Baud rate select

    RCSTA1bits.CREN = 1;      // Enable receive mode on the serial port

    // Turn on the serial port
    TXSTA1bits.TXEN = 1;      // Enable transmitter
    RCSTA1bits.SPEN = 1;      // Enable receiver

  // I2C configuration
    ANSELCbits.ANSC3 = 0;     // Disable analog mode for SCA
    ANSELCbits.ANSC4 = 0;     // Disable analog mode for SDA

    SSPCON1bits.SSPM = 0b1000;// Configure the serial port for i2c mode
    SSPADD = 0x27;            // Set the speed to 100kHz
    SSPCON1bits.SSPEN = 1;    // Enable the I2C port
    SSP1CON2bits.PEN = 1;     // and make sure it is stopped
}

unsigned int logCount = 0;     // Number of samples that the logger has acquired

int logInterval = 10; // Number of seconds between measurements

char logging = 0;     // Specifies whether we are actively logging or not

int intervalCounter = 0;

// This function is called repeatedly
void loop(void) {
    // Temporary storage variables
    unsigned int i;
    unsigned char temperature;
    unsigned char data;
    char buffer[50];

    // Check if the serial port has overflown, and clear the event if that happened.
    if (RCSTAbits.OERR) {
      RCSTA1bits.CREN = 0;
      RCSTA1bits.CREN = 1;
    }

    // Check if there is serial data waiting for us
    if(PIR1bits.RC1IF) {
        data = RCREG1;

        // See if we got a command
        switch (data) {
          case 'm':       // Measure the current temperature
            // Read the temperature
            temperature = readTemperature();

            // Write the results out to a string
            sprintf(buffer,"The current temperature is: %u\n\r", temperature);

            // Then send that string to the serial port
            puts1USART(buffer);
            break;
          case 'b':       // Begin logging
            sprintf(buffer,"Logging started\n\r");
            puts1USART(buffer);
            logging = 1;
            break;
          case 'e':       // End logging
            sprintf(buffer,"Logging stopped\n\r");
            puts1USART(buffer);
            logging = 0;
            break;
          case 'd':       // Get the logged data
            sprintf(buffer,"Getting %u measurements:\n\r", logCount);
            puts1USART(buffer);
            for (i = 0; i < logCount; i++) {
                temperature = readEEPROM( i );
                // Write the results out to a string
                sprintf(buffer,"%u, ", temperature);

                // Then send that string to the serial port
                puts1USART(buffer);
            }
            sprintf(buffer,"\n\r", temperature);
            puts1USART(buffer);
            break;
          case 'r':       // Reset the logger, clearing all data
            sprintf(buffer,"Logger reset\n\r");
            logging = 0;
            logCount = 0;
            intervalCounter = 0;
            break;
        }
    }

    // Wait for one second
    wait(1);

    // If data logging is enabled, see if we should log some now.   
    if ( logging ) {
        // Record that another second has passed
        intervalCounter++;

        // If we have waited for enough time, measure the current temperature
        if (intervalCounter >= logInterval) {
            // Turn on the LED to signal that a log event is happening
            PORTAbits.RA0 = 1;

            // Reset the seconds counter
            intervalCounter = 0;

            // Read the temperature
            temperature = readTemperature();

            // And record it in the EEPROM
            writeEEPROM( logCount, temperature );

            // Then record that we have taken another sample
            logCount++;

            // And turn off the LED
            PORTAbits.RA0 = 0;
        }
    }

}
