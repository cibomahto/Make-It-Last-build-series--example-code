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

// Configure it to run at 4 MHz
#pragma config PLLCFG = OFF

// Disable the watchdog timer
#pragma config WDTEN = OFF


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
int measureSwitch(void);
void moveServo(unsigned int pos);

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


// Output a control signal to move the servo to a new position
void moveServo(unsigned int pos) {
    unsigned int a, i;

    // Turn on power to the servo
    PORTCbits.RC2 = 1;

    // Generate the control signal to move the servo
    for (i = 0; i < 80; i++) {
		// Turn on the servo and delay for a short time
        PORTCbits.RC3 = 1;
        for (a = 0; a < pos + 20; a++) {}

		// Turn off the servo and delay for a short time
        PORTCbits.RC3 = 0;
        for (a = 0; a < 800 - pos; a++) {}
    }

    // turn off power to the servo
    PORTCbits.RC2 = 0;
}


// This function is called once, when the microcontroller is turned on.
void setup( void ) {
  // Configure the oscillator to run at 4 MHz
    OSCCONbits.IRCF = 101;      // 4 MHz


  // ADC configuration
    // Configure the internal voltage regulator to output 2.048V
    // This voltage is stable over varying battery voltage, and is used
    // as a reference when measuring the sensors.
    VREFCON0bits.FVREN = 1;
    VREFCON0bits.FVRS = 0b10; // Positive reference = 2.048V

    // Set the ADC to use ground (0V) for the negative reference, and
    // the internal voltage regulator (2.048V) as the positive reference.
    ADCON1bits.NVCFG = 0;     // Negative reference = 0V
    ADCON1bits.PVCFG = 0b10;  // Positive reference = 2.048V

    ADCON2bits.ADFM=1;        // Right justify the results

    // Set the speed that the ADC should capture data
    ADCON2bits.ACQT=0b111;    // Acquisition time
    ADCON2bits.ADCS=0b001;    // Fosc/8

    // Tell the ADC to use pin 13 (input 14) as the input, and turn on the ADC
    ADCON0bits.ADON = 1;      // Turn the ADC on

  // Clock timer configuration
    // Configure the first timer to use the secondary oscillator
    // (Datasheet, page 162)
    T1CONbits.TMR1CS = 0b10;  // Use the external clock
    T1CONbits.T1OSCEN = 1;    // Enable timer 1 oscillator
    T1CONbits.T1CKPS = 0b00;  // Disable the prescaler
    T1CONbits.NOT_T1SYNC = 1; // Disable clock sync
    PIE1bits.TMR1IE = 1;      // Enable timer1 interrupt
    T1CONbits.TMR1ON = 1;     // And turn on the timer

    // Enable global interrupts
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

  // I/O pin confiugration

    // Make the status LED an output
    TRISAbits.TRISA0 = 0;     // Make the LED pin an output
    PORTAbits.RA0 = 0;        // but turn it off to disable the LED

    // Make the servo power switch an output
    TRISCbits.TRISC2 = 0;
    ANSELCbits.ANSC2 = 0; 
    PORTCbits.RC2 = 0;        // and turn it off for now

    // Make the servo control signal an output
    TRISCbits.TRISC3 = 0;
    PORTCbits.RC3 = 0;

    // Make the capacitive sensor an input
    TRISAbits.TRISA1 = 1;
    ANSELAbits.ANSA1 = 1;
}


// This function is called repeatedly
int lastState, in;
char firstRun = 1;

void loop(void) {

    // Read in the value of the capacitive sensor
    in = measureSwitch();

    // If this is our first time running, ignore the input we just read
    if (firstRun == 1) {
        firstRun = 0;
        lastState = in;
    }

    // If the input is differente from the last time (we sensed a
    // touch!), move the servo.
    if (in != lastState) {
        lastState = in;

        if (lastState == 0) {
            moveServo(60);
        } else {
            moveServo(10);
        }
    }
}


// This function is called whenever a high interrupt occurs. For the
// datalogger project, this only happens when the EUSART (serial port)
// receives a character.
#pragma interruptlow receive_interrupt
void receive_interrupt (void)
{
    // If the interrupt was caused by the timer overflowing, toggle the LED.
    if (PIR1bits.TMR1IF == 1) {
        TMR1H = 128;

        if (LATAbits.LATA0 != 1) {
            LATAbits.LATA0 = 1;
        }
        else {
            LATAbits.LATA0 = 0;
        }

        // Clear the interrupt
        PIR1bits.TMR1IF = 0;
    }
}


// Capacitive sensing routine from PIC 18lf22 datasheet, page 326

#define COUNT 50                     //@ 4MHz = 125uS.
#define DELAY for(i=0;i<COUNT;i++)
#define OPENSW 1000                   //Un-pressed switch value
#define TRIP 100                      //Difference between pressed

#define HYST 65                       //amount to change
                                      //from pressed to un-pressed
#define PRESSED 1
#define UNPRESSED 0

int measureSwitch(void)
{
    unsigned int Vread;           //storage for reading
    int i;

    // First, select input 
    ADCON0bits.CHS = 1;

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
        return PRESSED;
    }
    else if(Vread > OPENSW - TRIP + HYST)
    {
        return UNPRESSED;
    }
}
