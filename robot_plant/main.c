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
#pragma config WDTEN = OFF

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

int measureSwitch(void);

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


void moveServo(unsigned int pos) {
    unsigned int a, i;

    // turn on servo
    PORTBbits.RB4 = 1;

    // move servo
    for (i = 0; i < 80; i++) {

        PORTBbits.RB5 = 1;
        for (a = 0; a < pos + 20; a++) {}

        PORTBbits.RB5 = 0;
        for (a = 0; a < 800 - pos; a++) {}
    }

    // turn off servo
    PORTBbits.RB4 = 0;
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
    unsigned int a, b, c;


    // Configure the oscillator to run at 4 MHz
    OSCCONbits.IRCF = 101;      // 4 MHz

    TRISAbits.TRISA0 = 0;     // Make the LED pin an output
    PORTAbits.RA0 = 0;        // but turn it off to disable the LED

    TRISAbits.TRISA1 = 0;     // Make the LED pin an output
    PORTAbits.RA1 = 0;        // but turn it off to disable the LED

  // ADC configuration
    // Configure pin 13 (port C, pin 2) as an input
//    TRISCbits.TRISC2 = 1;	  // Make the pin an input
//    ANSELCbits.ANSC2 = 1;     // Specifically, an analog input
//    ADCON0bits.CHS = 14;      // Select pin 13 as the input

    TRISAbits.TRISA5 = 1;
    ANSELAbits.ANSA5 = 1;
    ADCON0bits.CHS = 4;

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
    ADCON2bits.ACQT=0b111;    // Acquisition time
    ADCON2bits.ADCS=0b001;    // Fosc/8

    // Tell the ADC to use pin 13 (input 14) as the input, and turn on the ADC
    ADCON0bits.ADON = 1;      // Turn the ADC on


  // Serial port configuration
    TRISCbits.TRISC6 = 0;	  // Make TX pin an output
    TRISCbits.TRISC7 = 1;	  // and RX pin an input
    ANSELCbits.ANSC7 = 0;     // Specifically, a digital input

    // Configure the serial port to run at 9600 baud
    // (see manual, page 275)

    // for 4 MHz clock
    SPBRG1 = 25;    //9600
    TXSTA1bits.BRGH = 0;      // Baud rate select
    BAUD1CONbits.BRG16 = 1;

    PIE1bits.RCIE = 1;
    PIE1bits.RC1IE = 1;       // Turn on interrupts on serial receive
    BAUDCON1bits.WUE = 1;     // Wake up the processor when a serial 

    // Configure the first timer to use the secondary oscillator
    // (Datasheet, page 162)
    T1CONbits.TMR1CS = 0b10;  // Use the external clock
    T1CONbits.T1OSCEN = 1;    // Enable timer 1 oscillator
    T1CONbits.T1CKPS = 0b00;  // Disable the prescaler
    T1CONbits.NOT_T1SYNC = 1; // Disable clock sync
    PIE1bits.TMR1IE = 1;      // Enable timer1 interrupt
    T1CONbits.TMR1ON = 1;     // And turn on the timer

    // Configure the second timer to use the primary oscillator
//    T1CONbits.TMR1CS = 0b10;  // Use the external clock
//    T1CONbits.T1OSCEN = 1;    // Enable timer 1 oscillator
//    T1CONbits.T1CKPS = 0b00;  // Disable the prescaler
//    T1CONbits.NOT_T1SYNC = 1; // Disable clock sync
//    PIE1bits.TMR1IE = 1;      // Enable timer1 interrupt
//    T1CONbits.TMR1ON = 1;     // And turn on the timer

    // CCP3

    TRISBbits.TRISB4 = 0;     // Make the servo switch an output
    ANSELBbits.ANSB4 = 0; 
    PORTBbits.RB4 = 0;        // but turn it off to disable


    TRISBbits.TRISB5 = 0;     // Make the servo an output
//    ANSELBbits.ANSB5 = 0; 
    PORTBbits.RB5 = 0;        // but turn it off to disable

    // Enable global interrupts
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    // Turn on the serial port
    RCSTA1bits.CREN = 1;      // Enable receive mode on the serial port
    TXSTA1bits.TXEN = 1;      // Enable transmitter
    RCSTA1bits.SPEN = 1;      // Enable receiver


    PORTBbits.RB4 = 1;        // but turn it off to disable
}



unsigned int logCount = 0;     // Number of samples that the logger has acquired
int logInterval = 10; // Number of seconds between measurements
char logging = 0;     // Specifies whether we are actively logging or not
int intervalCounter = 0;

// This function is called repeatedly
int lastState = 0, in;
unsigned char buffer[30];

void loop(void) {
    in = measureSwitch();

/*
    if (lastState == 0) {
        lastState = 1;
    }
    else {
        lastState = 0;
    }
*/

    if (in != lastState) {
        lastState = in;

        if (lastState == 0) {
            LATAbits.LATA1 = 1;
//            PORTAbits.RA1 = 1;
            moveServo(60);
//            sprintf(buffer, "%u %u\r\n", TMR1H, TMR1L);
//            puts1USART(buffer);
        } else {
            LATAbits.LATA1 = 0;
//            PORTAbits.RA1 = 0;
            moveServo(10);
//            sprintf(buffer, "%u\r\n", in );
//            puts1USART(buffer);
        }

    }

}

unsigned char ledState = 0;

// This function is called whenever a high interrupt occurs. For the
// datalogger project, this only happens when the EUSART (serial port)
// receives a character.
#pragma interruptlow receive_interrupt
void receive_interrupt (void)
{
    if (PIR1bits.TMR1IF == 1) {
        // 
        TMR1H = 128;

        if (ledState != 1) {
            ledState = 1;
            LATAbits.LATA0 = 1;
//            PORTAbits.RA0 = 1;
//            PORTA = PORTA | 1<<0;
        }
        else {
            ledState = 0;
            LATAbits.LATA0 = 0;
//            PORTAbits.RA0 = 0;
//            PORTA = (PORTA & 0xFE);
        }

        // Clear the interrupt
        PIR1bits.TMR1IF = 0;
    }
    else if (PIR1bits.RC1IF == 1) {
        // Serial receive interrupt

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
}


// PIC 18lf22 datasheet, page 326

//#define COUNT 500                     //@ 8MHz = 125uS.
#define COUNT 1050                     //@ 4MHz = 125uS.
//#define COUNT 650                     //@ 4MHz = 125uS.
#define DELAY for(i=0;i<COUNT;i++)
#define OPENSW 1000                   //Un-pressed switch value
#define TRIP 300                      //Difference between pressed
                                      //and un-pressed switch
#define HYST 65                       //amount to change
                                      //from pressed to un-pressed
#define PRESSED 1
#define UNPRESSED 0


int measureSwitch(void)
{
    unsigned int Vread;           //storage for reading
    unsigned int switchState;
    int i;

    //assume CTMU and A/D have been setup correctly
    //see Example 25-1 for CTMU & A/D setup

    CTMUCONHbits.CTMUEN = 1;      // Enable the CTMU
    CTMUCONLbits.EDG1STAT = 0;    // Set Edge status bits to zero
    CTMUCONLbits.EDG2STAT = 0; 
    CTMUCONHbits.IDISSEN = 1;     //drain charge on the circuit
    DELAY;  //wait 125us
    CTMUCONHbits.IDISSEN = 0;     //end drain of circuit

    CTMUCONLbits.EDG1STAT = 1;    //Begin charging the circuit
                                  //using CTMU current source
    DELAY;                        //wait for 125us
    CTMUCONLbits.EDG1STAT = 0;    //Stop charging circuit

    PIR1bits.ADIF = 0;            //make sure A/D Int not set
    ADCON0bits.GO=1;              //and begin A/D conv.
    while(!PIR1bits.ADIF);        //Wait for A/D convert complete

    Vread = ADRES;                //Get the value from the A/D

    CTMUCONHbits.CTMUEN = 0;      // Disable the CTMU

    if(Vread < OPENSW - TRIP) 
    {
        return 255;
    }
    else if(Vread > OPENSW - TRIP + HYST)
    {
        return 0;
    }
}
