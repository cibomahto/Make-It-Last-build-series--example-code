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
#pragma config FOSC = INTIO67		// Use internal clock, don't output 
//#pragma config FOSC = INTIO7        // Use internal clock, output on RA6

// Configure it to run at 16 MHz
#pragma config PLLCFG = OFF

// Allow the program to turn on the watchdog timer. The watchdog is a
// a special feature of the processor, that runs separately from the main
// program and can be used to wake up the processor after a certain amount of time.
#pragma config WDTEN = SWON

// Set the watchdog timer prescaler to 1.
#pragma config WDTPS = 256


// Fix a compiler bug
#undef INTCON
#undef INTCONbits

/////////////////////////////////////////////////////////////////////////////
// Function declarations
// Declare any user functions that you want to use here
/////////////////////////////////////////////////////////////////////////////
void main (void);
void setup(void);
void loop(void);

void receive_interrupt(void);

unsigned char receiveCharacter = 0;    // Stores the last character that was
                                       // received by the serial port
unsigned char stayAwakeCount = 0;      // 5 second countdown to keep the
                                       // chip from sleeping

/////////////////////////////////////////////////////////////////////////////
// Function definitions
// Define any user functions that you want to use here
/////////////////////////////////////////////////////////////////////////////

// Configure the high interrupt to call the high_isr() function
// From MPLAB C18 C Compiler Users Guide, Page 29
#pragma code low_vector=0x08
void low_interrupt (void)
{
_asm GOTO receive_interrupt _endasm
}
#pragma code /* return to the default code section */



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
    // If we need to stay awake for a while, to receive serial data, delay one
    // second by counting.
    if (stayAwakeCount > 0) {
        int i;
        int j;

        // It takes the microcontroller approximately one second to run through
        // this counting loop
//        for (i = 0; i < 255; i++) {
        for (i = 0; i < 64; i++) {
            for (j = 0; j < 750; j++) {
            }
        }

        // Decrement the counter, to reflect the extra second we stayed awake.
        stayAwakeCount -= 1;
    }

    // Otherwise, turn on the watchdog timer, then go to sleep!
    else {
        // Put the serial receive pin in wake-on-receive mode, so that a
        // message from the computer will bring it out of sleep.
        BAUDCON1bits.WUE = 1;

        WDTCONbits.SWDTEN = 1;  // Enable the software watchdog
        Sleep();                // Sleep until the watchdog wakes us up
        WDTCONbits.SWDTEN = 0;  // Disable the software watchdog
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


unsigned int readTemperature(void)
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


//    OSCCONbits.IRCF = 111;      // 16 MHz
//    OSCCONbits.IRCF = 110;      // 8 MHz
    OSCCONbits.IRCF = 101;      // 4 MHz


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

    ADCON2bits.ADFM=1;        // Right Justified

    // Set the speed that the ADC should capture data
    // for 16 MHz clock
//    ADCON2bits.ACQT=0b111;    // Acquisition time
//    ADCON2bits.ADCS=0b010;    // Fosc/32

    // for 4 MHz clock
    ADCON2bits.ACQT=0b111;    // Acquisition time
    ADCON2bits.ADCS=0b001;    // Fosc/8

    // Tell the ADC to use pin 13 (input 14) as the input, and turn on the ADC
    ADCON0bits.CHS = 14;      // Select pin 13 as the input
    ADCON0bits.ADON = 1;      // Turn the ADC on


  // Serial port configuration
    TRISCbits.TRISC6 = 0;	  // Make TX pin an output
    TRISCbits.TRISC7 = 1;	  // and RX pin an input
    ANSELCbits.ANSC7 = 0;     // Specifically, an analog input

    // Configure the serial port to run at 9600 baud
    // (see manual, page 275)

    // for 16 MHz clock
//    SPBRG1 = 25;    
//    TXSTA1bits.BRGH = 0;      // Baud rate select
//    BAUD1CONbits.BRG16 = 0;
    
    // for 4 MHz clock
    SPBRG1 = 25;
    TXSTA1bits.BRGH = 0;      // Baud rate select
    BAUD1CONbits.BRG16 = 1;

    PIE1bits.RCIE = 1;
    PIE1bits.RC1IE = 1;       // Turn on interrupts on serial receive
    BAUDCON1bits.WUE = 1;     // Wake up the processor when a serial 

    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    // Turn on the serial port
    RCSTA1bits.CREN = 1;      // Enable receive mode on the serial port
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
    if (receiveCharacter != 0) {

        // See if we got a command
        switch (receiveCharacter) {
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
            sprintf(buffer,"Getting %u measurements: ", logCount);
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
            puts1USART(buffer);
            logging = 0;
            logCount = 0;
            intervalCounter = 0;
            break;
        }

        receiveCharacter = 0;
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


// This function is called whenever a high interrupt occurs. For the
// datalogger project, this only happens when the EUSART (serial port)
// receives a character.
#pragma interruptlow receive_interrupt
void receive_interrupt (void)
{
    // Read the character in from the serial module. If the serial port just
    // woke up the microcontroller, this will be garbage.
    receiveCharacter = RCREG;

    // If we just woke up from sleeping, discard the first character because
    // it is probably corrupted
    if (stayAwakeCount == 0) {
        receiveCharacter = 0;
    }

    // Start a counter to wait 5 seconds before sleeping, in order to give
    // the user time to send a serial command
    stayAwakeCount = 5;
}