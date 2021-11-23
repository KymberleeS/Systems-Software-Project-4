/********************************************************************
 FileName:     	helloLCD.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18F2580 Microcontrollers

 Compiler:		XC8
 Company:		Microchip Technology, Inc.
 Authors:		Harrison Wine
				Donald Lanoux
				Bradley Crutchfield
				Kymberlee Sables

 File Description: This program interfaces with a 16x2 LCD from Sparkfun.
 The LCD-00255 is a 5 Volt black-on-green backlit LCD that has a parallel
 interface. The interface is controlled with an HD44780 compatible controller.
 The Sparkfun website includes the board datasheet: GDM1602K. The board datasheet
 mentions the use of the KS0066U controller which is equivalent to the HD44780.
 
********************************************************************/

#include <xc.h>
#include <stdlib.h>

// PIC18F2580 Configuration Bit Settings
// Configuration bits are stored in SFR configuration bytes

// CONFIG1H
#pragma config OSC = IRCIO67     // Oscillator Selection bits (Internal oscillator block, port function on RA6, port function on RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (VBOR set to 2.1V)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = 1024     // Boot Block Size Select bit (1K words (2K bytes) boot block)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)


/*Need preprocessor definition to use compiler delay function below. The
compiler needs to know what the speed is that the MCU is running at. See
 section 2.7.1 of the datasheet for info about the default speed and
 changing it. */

#define _XTAL_FREQ 1000000 //This is the speed the PIC is running at

//-----------------------DATA MEMORY FOR LCD OPERATION------------------------

#define number 0x30     //Offset in ASCII table to get to where numbers start

#define	LCD_RS LATAbits.LA6      // LCD Register Select.
//#define	LCD_RW RA4      // LCD Read/Write.
#define LCD_EN LATAbits.LA7      // LCD Enable.
#define LCD_DATA LATC  // RC0-RC3 data to LCD in 4-bit mode. RC4-RC7 not used.

#define	LCD_STROBE()    ((LCD_EN = 1),(LCD_EN=0))  //C macro substitution with arguements
                        // LCD strobe sends a quick pulse to the LCD enable.
                        // The sequence sets LA7 and then clears LA7.
                        // The pulse will last 4 clock cycles at 1MHz so
                        // 4usecs. See GDM1602K datasheet for enable write time.
                        // Minimum pulse time is 500ns.

unsigned char b0;       //counter for displaying numbers

/** SUBROUTINE DECLARATIONS********************************/

void Initialize(void);
void lcd_write(unsigned char c);
void lcd_clear(void);
void lcd_puts(const char *s);
void lcd_putch(char c);
void lcd_goto(unsigned char c);


int hexToDec(char* hexInput);
int secondsToHour(int seconds);
int secondsToMinutes (int seconds);
int convertTimeFormat(int hour);
int timeFormatFlag(int hour);

/*----------------------------------------------------------
	Subroutine: main
-----------------------------------------------------------*/

void main(void)
{
Initialize(); //Initialize subroutine runs only once
while(1)   //infinite loop
{
	lcd_clear();			// Clear LCD screen
	lcd_goto(0);			// select first line
	for (b0=0; b0<10; b0=b0+1)      // Create counting loop
		{
		lcd_putch(b0+number);   // Display count on LCD
		__delay_ms(250);	// Delay 250 msec to see each count
		}

	lcd_goto(0x40);                 // Select second line
	lcd_puts("Hello World!");   // Display Text
	__delay_ms(500);		// Delay for 1/2 second to read display
	__delay_ms(500);		// Delay for 1/2 second to read display
	__delay_ms(500);		// Delay for 1/2 second to read display
	__delay_ms(500);		// Delay for 1/2 second to read display
}

}

/*----------------------------------------------------------
	Subroutine: INITIALIZE
----------------------------------------------------------*/

void Initialize(void)
{
   //Set the GPIO pin configurations

    TRISAbits.RA6 = 0; // Sets Port A bit to output
    TRISAbits.RA7 = 0; // Sets Port A bit to output
    TRISC = 0; // Set Port C bits for outputs

                  //We will use the LCD in 4 bit mode, instead of 8 bit mode.
                  //This will save us 4 i/o pins
                  //See page 46 (Figure 24) of the Hitachi datasheet to
                  //understand the required steps to initialize 4 bit mode

	char init_value;        //variable to use for initializing


    LCD_RS = 0;             //LCD control pin set to low
	LCD_EN = 0;             //LCD control pin set to low
//	LCD_RW = 0;             //LCD control pin set to low

	init_value = 0x03;      //Required for initial FUNCTION SET. see pg 46

	__delay_ms(15);		//wait 15mSec after power applied,
	LCD_DATA = init_value;  //set up the intial LCD data, See pg 46
	LCD_STROBE();           //Burst the Enable line to write the data
	__delay_ms(10);              //At least 4.1ms delay. See pg 46
	LCD_STROBE();           //Burst the Enable line again to write the data
	__delay_ms(10);              //At least 4.1ms delay. See pg 46
	LCD_STROBE();           //Burst the Enable line again to write the data
	__delay_ms(10);              //At least 4.1ms delay. See pg 46
    
	LCD_DATA = 0x02;	        // Function Set = Four bit mode
	LCD_STROBE();           //Burst the Enable to write the 4 bit mode cmd
                                //From now on all writes must be done using 2
                                //nibbles. This will send one byte in 2 pieces.
                                // The high nibble followed by the low nibble.
                                // To do this we use the lcd_write function.
	__delay_ms(10);              //At least 4.1ms delay. See pg 46

        //See pg 46 for the correct sequence of these initialization words.
	lcd_write(0x28);        // Function Set with 4 bits (2) and
                                // character font as 5x8 dots with 2 lines (8)
                                //N = 1 (2 lines) and F = 0 (5x8 Font), pg 25
	lcd_write(0x0F);        //Set display mode.
                                //0x08 AND with these bits for display set up:
                                // Bit 2 set = Display On
                                // Bit 1 set = Cursor On
                                // Bit 0 set = Cursor Blink
	lcd_clear();            // Call the clear screen function
	lcd_write(0x06);        // Set entry mode.
                                //0x04 AND with these bits for entry mode:
                                // Bit 1 set = increment the character address
                                // when a character code us written to DDRAM
                                //Bit 0 clear = no shift in display.
        //LCD Initialization is done
}
void lcd_write(unsigned char c)   //write a byte to the LCD in 4 bit mode
                                  //we send a byte to PORTC but only RC0-RC3
                                  //have data on them. We mask 0s on the other
                                  //pins for RC4-RC7.
                                  //This function is used to write commands,
                                  //addresses, and display characters to the LCD
{
	__delay_ms(1);                          // 1 millisecond delay
	LCD_DATA = ( ( c >> 4 ) & 0x0F );   // Put upper nibble of character
                                            // on RC0-RC3
	LCD_STROBE();                       // Pulse the LCD enable
	LCD_DATA = ( c & 0x0F );            // Put lower nibble of character
                                            // on RC0-RC3
	LCD_STROBE();                       // Pulse the LCD enable
}

void lcd_clear(void)  //Clear and home the LCD
{
	LCD_RS = 0;         //Clear the LCD register select line to send
                            //an instruction as opposed to data (when RS = 1).
	lcd_write(0x01);    //Instruction to clear display
	__delay_ms(2);          //Wait 2 millliseconds for clear to complete
}

void lcd_puts(const char *s)   //write a string of chars to the LCD
                                // s is a pointer to the string that is
                                //brought into the function lcd_puts().
                                //Think of *s as "what s points at".
                                //The variable is a const (constant) because
                                //we are bringing in a string of characters in
                                //quotes and not a variable. The declaration
                                // knows the number of constant characters to
                                //assume for *s based on the string in quotes.
{
	LCD_RS = 1;	    //Set the RS bit to allow writing display data
	while(*s)           //Check to see if all of the characters in the
                            //string have been written. If not keep
                            //writing them.
	lcd_write(*s++);    //Write each character that s points to and then
                            //increment the string pointer to the next
                            //character. Go back to the while(*s) loop
                            //until the end of the string has been reached. The
                            // end of a string is always a null character.
                            //A null character is like a zero which will
                            //terminate the while loop.
}

void lcd_putch(char c)   //write one character to the LCD
{
	LCD_RS = 1;	//Set the RS bit to allow writing display data
	lcd_write(c);   //Send the character c to the lcd_write routine
}

void lcd_goto(unsigned char pos)    //Go to the specified position
{
	LCD_RS = 0;                 //Clear the LCD register select line
	lcd_write(0x80+pos);        //0x80 is the instruction to move the cursor
                                    //to a specific address. The addresses for
                                    //the first line are 00-0F. The addresses
                                    //for the second line are 40-4F. So there
                                    //are 16 characters on each line.
                                    //The addresses are always less than 7 bits
                                    //and so don't corrupt the bit for 0x80.
                                    //See the "Set DDRAM address" section of
                                    //the datasheet
}



// function that converts hex input to decimal
int hexToDec(char* hexInput) {
    return (int)strtol(hexInput, NULL, 16);
}

// function that converts seconds to the hour value
int secondsToHour(int seconds) {
    return seconds / 3600;
}

// function that converts seconds to the minute value
int secondsToMinutes (int seconds) {
    int remainder = seconds % 3600;
    return remainder / 60;
}

// function that converts 24 hour format to 12 hour format
int convertTimeFormat(int hour) {
    if ((hour > 12) && (hour <= 23)) {
        return hour -= 12;
    } else if (hour == 0) {
        return 12;
    }
    
    return hour;
}

// function that determines AM or PM for the 12 hour time format
int timeFormatFlag(int hour) {
    int flag;
    
    if (hour < 12) {
        flag = 65;
    } else {
        flag = 80;
    }
    
    return flag;
}
