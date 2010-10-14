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


#pragma config WDTEN = OFF       // Make sure that the internal watchdog timer
                                 // is off. This is a special 


/////////////////////////////////////////////////////////////////////////////
// Variable definitions
// Define any variables that you want to use in your program here
/////////////////////////////////////////////////////////////////////////////

#define NUMSAMPLES 100

unsigned char pattern[] = {1,2,4,8,16,32,64,128,64,32,16,8,4,2};
#define MAXPATTERN sizeof(pattern);
unsigned char position = 0;

unsigned char message[] = {
0b10000010,
0b11111110,
0b10010010,
0b00010000,
0b10010010,
0b11111110,
0b10000010,
0b00000000,
0b11010000,
0b10101000,
0b10101000,
0b10101000,
0b11110000,
0b10000000,
0b00000000,
0b01110000,
0b10001000,
0b10001000,
0b10001000,
0b10011000,
0b00000000,
0b10000010,
0b11111110,
0b00100000,
0b10101000,
0b11011000,
0b10001000,
0b00000000,
0b00000000,
0b10000010,
0b11111110,
0b10100010,
0b00100010,
0b00011100,
0b00000000,
0b00000000,
0b01111100,
0b10000010,
0b10000010,
0b10100010,
0b01100110,
0b00100000,
0b10000010,
0b11111110,
0b10010010,
0b00010000,
0b10010010,
0b11111110,
0b10000010,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
};
#define MAXMESSAGE sizeof(message);


/////////////////////////////////////////////////////////////////////////////
// Function declarations
// Define any user functions that you want to use here
/////////////////////////////////////////////////////////////////////////////


void puts1USART( char *data)
{
  do
  {  // Transmit a byte
    while(!TXSTA1bits.TRMT);
    TXREG1 = *data;      // Write the data byte to the USART2
//    TXREG1 = 'a';      // Write the data byte to the USART2
  } while( *data++ );
}

// Wait for 1 second
void waitASecond(void) {
    int i;
    int j;
    for (i = 0; i < 255; i++) {
        for (j = 0; j < 750; j++) {
        }
    }
}

// Wait a certain number of seconds
void wait( int time ) {
    int i;
	// Based on the clock speed, wait for some time.
    // really rough.
    for (i = 0; i < time; i++) {
        waitASecond();
	}
}


void setup(void) {
  // Set up the oscillator to do 16 MHz
  OSCCONbits.IRCF = 111;


  /* Make all bits on the Port B (LEDs) output bits.
   * If bit is cleared, then the bit is an output bit.
   */
  TRISA = 0;
  PORTA = 0x0;

  // Configure the internal voltage regulator to be 2.048V
  VREFCON0bits.FVREN = 1;
//  VREFCON0bits.FVRS = 0b01;	  // Positive reference = 1.024V
  VREFCON0bits.FVRS = 0b10;	// Positive reference = 2.048V


  // Set the ADC reference voltages
  ADCON1bits.NVCFG = 0;		// Negative reference = 0V
  ADCON1bits.PVCFG = 0b10;	// Positive reference = 2.048V

// This code block configures the ADC 
// for polling, clock and AN0 input.

  // Conversion start & polling for completion
  // are included.

  ADCON2bits.ADFM=1;          // Right Justified
  ADCON2bits.ACQT=0b111;      // Acquisition time
  ADCON2bits.ADCS=0b010;      // Fosc/32

  // Set port C, pin 2 as an input
  TRISCbits.TRISC2 = 1;
  ANSELCbits.ANSC2 = 1;


  // Select input 14 and turn on the ADC
  ADCON0bits.CHS = 14;
  ADCON0bits.ADON = 1;

// Turn on the EUSART
  TXSTA1 = 0;          // Reset USART registers to POR state
  RCSTA1 = 0;

  BAUD1CONbits.BRG16 = 0;

  TXSTA1bits.BRGH = 0; // Baud rate select
  SPBRG1 = 25;  	   // Assume 16MHz clock and 9600 baud serial
  //SPBRG1 = 1;

  PIE1bits.RC1IE = 0;	// Disable serial interrupts
  PIE1bits.TX1IE = 0;

  TXSTA1bits.TXEN = 1;  // Enable transmitter
  RCSTA1bits.SPEN = 1;  // Enable receiver

  // Wait until the ADC is set up
  while (ADCON0bits.GO == 1) {}
}

void loop(void) {
  unsigned char i;
  unsigned int count;

  char buffer[30];

  unsigned int results;


// Our example can do three different things, choose the one we want
#define DO_HELLO_WORLD 1
//#define DO_SCANNER 1
//#define DO_POV
//#define DO_ADC 1

#ifdef DO_HELLO_WORLD
	// Turn the LED on
	PORTAbits.RA0 = 1;

	// Create a short delay
    for( count = 0; count < 64000; count++) {}

	// Turn the LED off
	PORTAbits.RA0 = 0;

	// Create a short delay
    for( count = 0; count < 64000; count++) {}
#endif


#ifdef DO_SCANNER
	PORTA = pattern[position];
	position = (position + 1)%MAXPATTERN;

    for( count = 0; count < 10000; count++) {}
#endif

#ifdef DO_POV
    PORTA = message[position];
    position = (position + 1)%MAXMESSAGE;

    for( count = 0; count < 300; count++) {}
#endif

#ifdef DO_ADC
	results = 0;

	// read the ADC port a number of times
	for (i = 0; i < NUMSAMPLES; i++) {
    	// Read the ADC
    	ADCON0bits.GO = 1;
    	while (ADCON0bits.GO == 1) {}

		results += ((int)ADRESH << 8) + ADRESL;
	}

	// then average the results to get a final reading
	sprintf(buffer,"%u\n\r", results / NUMSAMPLES);

    // Copy it out into a buffer and send to serial
    puts1USART(buffer);

   // now wait for a minute
	wait(1);

#endif
}

void main (void)
{
  setup();

  while (1) {
    loop();
  }
}
