/*=============================================================================
 FileName:     	main.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18F2580 Microcontrollers
 Compiler:		XC8
 Company:		Microchip Technology, Inc.
 Authors:		Harrison Wine
				Donald Lanoux
				Bradley Crutchfield
				Kymberlee Sables

 File Description: This program allows the MCU to read from the DS3234 RTC which
 maintains track of time from seconds to years. The MCU and RTC communicate via
 SPI interface and that data is then broken down and displayed to the 
 HD44760 LCD. A button can be pressed to configure the RTC as well as switch
 from 12 hour mode to 24 hour (HH:MM:SSAM/PM or HH:MM:SS).
===============================================================================*/

#include <xc.h>
#include <stdlib.h>
#include <math.h>
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

#define LCD_RS LATCbits.LC0
#define LCD_EN LATCbits.LC1
#define CS LATCbits.LC2
#define LCD_DATA LATA
#define	LCD_STROBE()    ((LCD_EN = 1),(LCD_EN=0))  //C macro substitution with arguements
                        // LCD strobe sends a quick pulse to the LCD enable.
                        // The sequence sets LC1 and then clears LC1.
                        // The pulse will last 4 clock cycles at 1MHz so
                        // 4usecs. See GDM1602K datasheet for enable write time.
                        // Minimum pulse time is 500ns.

/** SUBROUTINE DECLARATIONS********************************/
void Initialize(void);
void lcd_write(unsigned char c);
void lcd_clear(void);
void lcd_puts(const char *s);
void lcd_putch(char c);
void lcd_goto(unsigned char c);
void spi_write(unsigned char x);
void set_timer(void);
void display_time(void);
void display_seconds(void);
void display_minutes(void);
void display_hours(void);
void display_AMPM(void);
void change_mode(void);
void __interrupt() changeTime(void);
unsigned char spi_comm(unsigned char spi_byte);
unsigned char spi_read(void);

//global flags
unsigned char flagAP     = 0x00;
unsigned char flag12hr   = 0x00;
unsigned char config_now = 0x00;
unsigned char change_now = 0x00;

/*----------------------------------------------------------
	Subroutine: main
-----------------------------------------------------------*/

void main(void){
Initialize(); //Initialize subroutine runs only once
while(1){ //infinite loop
    set_timer();
    change_mode();
    display_time();
}

}

/*----------------------------------------------------------
	Subroutine: INITIALIZE
----------------------------------------------------------*/

void Initialize(void){
   //Set the GPIO pin configurations
    /***WE WANT TO ONLY USE A0-A5, RESERVE PORTC FOR SPI***/
    TRISCbits.RC0 = 0; // Sets Port C bit to output
    TRISCbits.RC1 = 0; // Sets Port C bit to output
    //set SPI pins to input/output
    TRISCbits.RC5 = 0;
    TRISCbits.RC4 = 1;
    TRISCbits.RC3 = 0;
    //chip select output
    TRISCbits.RC2 = 0;

    TRISA = 0;
                  //We will use the LCD in 4 bit mode, instead of 8 bit mode.
                  //This will save us 4 i/o pins
                  //See page 46 (Figure 24) of the Hitachi datasheet to
                  //understand the required steps to initialize 4 bit mode

	char init_value;        //variable to use for initializing

    LCD_RS = 0;             //LCD control pin set to low
	LCD_EN = 0;             //LCD control pin set to low

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
    /***CONFIG SPI COMMICATION PERIPHERAL***/
     SSPSTAT = 0b10000000;       //Sets the data to be sampled at the end of the time
     SSPCON1 = 0b00110001;       //enable SPI, CLOCK idle high, fosc/16

     /***CONFIG INTERRUPTS***/
    TRISBbits.RB0 = 1;      //Sets as input
    INTEDG0 = 1;            //Sets INT0 to high to low
    INT0F = 0;              //Clears INT0 Flag
    INT0E = 1;              //Enables external interrupt
    GIE = 1;                //Enables unmasked interrupt to execute ISR

    TRISBbits.RB1 = 1;      //Sets as input
    INTEDG1 = 1;            //Sets INT1 to high to low
    INT1F = 0;              //Clears INT1 Flag
    INT1E = 1;              //Enables external interrupt
    GIE = 1;                //Enables unmasked interrupt to execute ISR
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

void spi_write(unsigned char x){
    unsigned char temp;
    temp = spi_comm(x);
}

unsigned char spi_read(void){
    unsigned char data;
    data = spi_comm(0x00);
    return data;
}

unsigned char spi_comm(unsigned char spi_byte){
    SSPBUF = spi_byte;
    while(!BF);
    return SSPBUF;
}

void set_timer(void){
    if(config_now == 0x01){
        CS = 0;
        spi_write(0x80);        //address & write enable
        spi_write(0x00);        //set seconds
        spi_write(0x30);        //set minutes;
        spi_write(0x52);        //set 24 hour time & hours //0b0110 0100
                                //range 00-23
        /*spi_write(0x01);        //set day
        spi_write(0x21);        //set date, range 01-31
        spi_write(0x11);        //set century/month
        spi_write(0x21);        //set year
        spi_write(0x40);        //enable osc, enable sqw
        spi_write(0x00);*/
        CS = 1;
        config_now = 0x00;
    }
}

void __interrupt() changeTime(void){
    if(INT0F == 1){
        INT0F = 0;
        if(PORTBbits.RB0 == 0){
           __delay_ms(1);
          if(PORTBbits.RB0 == 0){
               change_now = 0x01;
           }
       }

    }

    if(INT1F == 1){
      INT1F = 0;
        //use if conditions to make sure button is still being held down to
        //handle de-bounce issue
      if(PORTBbits.RB1 == 0){
          __delay_ms(1);
          if(PORTBbits.RB1 == 0){
                config_now = 0x01;
          }
       }
    }

}

void display_time(void){
    lcd_goto(0x00);
    display_hours();
    lcd_putch(':');
    display_minutes();
    lcd_putch(':');
    display_seconds();
    display_AMPM();
}

void display_seconds(void) {
    unsigned char sec;
    unsigned char sec10;

    CS = 0;             // select RTC
    spi_write(0x00);    // set clock to read at 0x00 (seconds register)
    sec = spi_read();   // reads value in the seconds register
    CS = 1;             // deselect RTC

    // get value of upper nibble (10s place of seconds value)
    sec10 = (sec >> 4);
    // get value of lower nibble (1s place of seconds value)
    sec = sec & 0x0F;

    // display on the LCD
    lcd_putch(sec10 + number);
    lcd_putch(sec + number);
}

// function to display minutes
void display_minutes(void) {
    unsigned char min;
    unsigned char min10;

    CS = 0;                     // select RTC
    spi_write(0x01);            // set clock to read at 0x01 (minutes register)
    min = spi_read();           // reads value in the minutes register
    CS = 1;                     // deselect RTC

    min10 = min >> 4;           // get value of upper nibble (10s place of minutes value)
    min = min & 0x0F;           // get value of lower nibble (1s place of minutes value)

    lcd_putch(min10 + number);  // display 10s digit on the LCD
    lcd_putch(min + number);    // display 1s digit on the LCD
}

void display_hours(void) {
    unsigned char hour;
    unsigned char hour10;
    unsigned char bit6;
    unsigned char bit5;

    CS = 0;             // select RTC
    spi_write(0x02);    // set RTC to read from 0x02 (hours register)
    hour = spi_read();  // store value of hours register
    CS = 1;             // deselect RTC

    bit6 = hour >> 6;   //

    if(bit6){
        //12 hr mode
        flag12hr = 0x01;
        // get value of upper nibble (10s place of hours value)
        // 0x04 - since according to the data sheet, 10hr can be extracted from bit 4
        // still shift to the left to remove the last 4 zeros
        hour10 = (hour >> 4) & 0x01;
        bit5 = (hour >> 5) & 0x01;
        flagAP = bit5;  //set flag for use in other functions
        // get value of lower nibble (1s place of hour value)
        hour = hour & 0x0F;

        // display on the LCD
        lcd_putch(hour10 + number);
        lcd_putch(hour + number);
    }
    else{
        //24hr mode
        flag12hr = 0x00;
        flagAP = 0x00;
        bit5 = (hour >> 5) & 0x01;
        if(bit5 == 0x00){
            //20 hr not set get hour10 & hr
            hour10 = (hour >> 4) & 0x01;
            hour = hour & 0x0F;
            lcd_putch(hour10 + number);
            lcd_putch(hour + number);
        }
        else{
            hour = hour & 0x0F;
            lcd_putch(bit5 + number + 0x01); //value is just of 1, need to add 1 so that it display 2 for when time is in the 20hrs
            lcd_putch(hour + number);
        }
    }
}

void display_AMPM(void){
    if(flag12hr){
        if(flagAP){
            lcd_puts("PM");
        }
        else{
            lcd_puts("AM");
        }
    }
    else{
        lcd_putch(0x20);
        lcd_putch(0x20);
    }
}

void change_mode(void){
    if(change_now == 0x01){
        change_now = 0x00;
        unsigned char hour_data;
        unsigned char clock_mode;
        unsigned char bit5;
        unsigned char hours_count = 0x00;
        unsigned char write_data  = 0x00;

        CS = 0;
        spi_write(0x02);
        hour_data = spi_read();
        CS = 1;

        clock_mode = hour_data >> 6;
        if(clock_mode){
            hours_count += hour_data & 0x0F;
            hours_count += ((hour_data >> 4) & 0x01) * 0x0A;
            bit5 = (hour_data >> 5) & 0x01;
            if(bit5){
              if(hours_count != 0x0C){
                  hours_count += 0x0C;
              }
            }
            if(hours_count < 0x14){
                if(hours_count > 0x09){
                    hours_count -= 0x0A;
                    write_data += 0x10;
                }
            }
            else{
                hours_count -= 0x14;
                write_data += 0x20;
            }
		
            write_data += hours_count & 0x0F;
            if(hour_data == 0x52){
                write_data = 0x00;
            }
        }
        else{//clock is in 24hr mode; convert 24 to 12 hr mode
            hours_count += hour_data & 0x0F;                    // gets right most nibble value (3)
            hours_count += ((hour_data >> 4) & 0x01) * 0x0A;    // gets hour10 bit and multiply by ten
            hours_count += (hour_data >> 5) * 0x14;             // gets hour20 bit and multiply by twenty
            switch(hours_count){
                case 1:                         // 1 am
                    write_data = 0b01000001;
                    break;
                case 2:                         // 2 am
                    write_data = 0b01000010;
                    break;
                case 3:                         // 3 am
                    write_data = 0b01000011;
                    break;
                case 4:                         // 4 am
                    write_data = 0b01000100;
                    break;
                case 5:                         // 5 am
                    write_data = 0b01000101;
                    break;
                case 6:                         // 6 am
                    write_data = 0b01000110;
                    break;
                case 7:                         // 7 am
                    write_data = 0b01000111;
                    break;
                case 8:                         // 8 am
                    write_data = 0b01001000;
                    break;
                case 9:                         // 9 am
                    write_data = 0b01001001;
                    break;
                case 10:                        // 10 am
                    write_data = 0b01010000;					
                    break;
                case 11:                        // 11 am
                    write_data = 0b01010001;
                    break;
                case 12:                        // 12 pm
                    write_data = 0b01110010;
                    break;
                case 13:                        // 1 pm
                    write_data = 0b01100001;
                    break;
                case 14:                        // 2 pm
                    write_data = 0b01100010;    
                    break;
                case 15:                        // 3 pm
                    write_data = 0b01100011;    
                    break;
                case 16:                        // 4 pm
                    write_data = 0b01100100;
                    break;
                case 17:                        // 5 pm
                    write_data = 0b01100101;
                    break;
                case 18:                        // 6 pm
                    write_data = 0b01100110;
                    break;
                case 19:                        // 7 pm
                    write_data = 0b01100111;
                    break;
                case 20:                        // 8 pm
                    write_data = 0b01101000;
                    break;
                case 21:                        // 9 pm
                    write_data = 0b01101001;
                    break;
                case 22:                        // 10 pm
                    write_data = 0b01110000;
                    break;
                case 23:                        // 11 pm
                    write_data = 0b01110001;
                    break;
                default:                        // 12 am
                    write_data = 0b01010010;
            }
        }
        
        CS = 0;                                     //select RTC
        spi_write(0x82);                            //Give it the write command to the hr register
        spi_write(write_data);                      //write modified configuration
        CS = 1;                                     //deselect RTC
    }
}
