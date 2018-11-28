/*
 * Name: Josephine Ruiter, Abbey Dengler
 * Class: EGR226
 * Professor: Zuidema
 * Final Project
 * Date: November, 2018
 *
 *  */

#include "msp.h"
#include <stdio.h>

void ADC14_init(void); // initialize ADC
void PortADC_init(void); //initialize ADC14 port
void systick_start(void); //prototype for initializing timer
void delay_ms(unsigned); //function prototype for delaying for x ms
void readtemp(); //function for reading the temp in C

//Functions For LCD
void timedisplay(char *line2); //prints the temperature, is passed nADC
void LCD_init(void);  //initializes the LCD
void delay_microsec(unsigned microsec);
void PulseEnablePin(void); //sequences the enable pin
void pushNibble(uint8_t nibble);  //puts one nibble onto data pins
void pushByte(uint8_t byte);    //pushes most significant 4 bits to data pins with the push nibble function
void write_command( uint8_t command); //writes one bit of command by calling pushByte() with the command parameter
void dataWrite(uint8_t data);   //will write one bit of data by calling pushByte()

static volatile uint16_t result; //vars used in temp reading
float nADC;
char line2[50];

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
    systick_start(); //Start up Systick

    LCD_init(); //LCD initialization
    ADC14_init();
    PortADC_init();

    while(1)
    {

    }

}



//FUCTIONS
//---------------------------------------------------------------------------------------
//Starts the systick timer for LCD delays
void systick_start() //initialize timer
{
    SysTick-> CTRL = 0; //off
    SysTick-> LOAD = 0xBB8; //1ms+clock
    SysTick-> VAL = 0; //reset count
    SysTick-> CTRL = 5; //enable with interrupt

}

//-----------------------------------------------------------------------------------------------------------------------------
//takes in amount of mili seconds to delay
void delay_ms(unsigned ms) //function for delaying for x ms. takes in ms
{
    SysTick-> LOAD = (3000* ms)-1; //1ms+clock
    SysTick-> VAL = 0; //reset count
    while((SysTick-> CTRL & 0x10000) == 0) {} ;  //function given in lab

}
//------------------------------------------------------------------------------------------------------------------------------
//takes in micro seconds to delay
void delay_microsec(unsigned microsec) //function for delaying for x ms. takes in ms
{
    SysTick-> LOAD = (300* microsec)-1; //1micros+clock
    SysTick-> VAL = 0; //reset count
    while((SysTick-> CTRL & 0x10000) == 0) {} ;  //function given in lab

}
//--------------------------------------------------------------------------------------------------------------------------------
//initalize the LCD
void LCD_init(void)  //initializes the LCD
{
    P6-> SEL0 &= ~0b11110011;  //BITS 0,1,4,5,6,7
    P6-> SEL1 &= ~0b11110011;
    P6-> DIR |= 0b11110011; //output
    P6-> OUT &= ~0b11110011;  //set output 0

    write_command(3);   //from lab appendix
    delay_ms(100);
    write_command(3);
    delay_microsec(200);
    write_command(3);
    delay_ms(100);

    write_command(2);
    delay_microsec(100);
    write_command(2);
    delay_microsec(100);

    write_command(8);
    delay_microsec(100);
    write_command(0x0F);
    delay_microsec(100);
    write_command(1);
    delay_microsec(100);
    write_command(6);
    delay_ms(10);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PulseEnablePin(void) //sequences the enable pin- kandalaft code
{
    P6-> OUT &= ~BIT1; //set pulse to 0V
    delay_microsec(10);
    P6-> OUT |= BIT1;
    delay_microsec(10);
    P6-> OUT &= ~BIT1; //set pulse to 0V
    delay_microsec(10);

}

//---------------------------------------------------------------------------------------------------------------------------------
void pushNibble(uint8_t nibble)  //puts one nibble onto data pins - kandalaft code
{
    P6-> OUT &= ~0xF0;  //clear p4.4-4.7
    P6-> OUT |= (nibble & 0x0F)<<4; //sets data pins  p4.4-4.7

    PulseEnablePin();

}

//----------------------------------------------------------------------------------------------------------------------------------
void pushByte(uint8_t byte)    //pushes most significant 4 bits to data pins with the push nibble function - kandalaft code
{
    uint8_t nibble;

    nibble=(byte & 0xF0)>>4;
    pushNibble(nibble);

    nibble=(byte & 0x0F);
    pushNibble(nibble);
    delay_microsec(100);
}
//-------------------------------------------------------------------------------------------------------------------------------------
void write_command( uint8_t command) //writes one bit of command by calling pushByte() with the command parameter
{
    P6-> OUT &= ~BIT0;
    pushByte(command);

}
//--------------------------------------------------------------------------------------------------------------------------------------
void dataWrite(uint8_t data) //will write one bit of data by calling pushByte()
{
    P6-> OUT |= BIT0;
    pushByte(data);
}
//--------------------------------------------------------------------------------------------------------------------
void timedisplay(char *line2) //prints the temperature, is passed nADC
{
      write_command(0b00000001); //reset display
      char line1[]= "Temperature is:";


     int i=0;
     while(line1[i] != '\0')
     {
         if (line1[i] != '\0')
             dataWrite(line1[i]);
         i++;
     }

     write_command(0b11000000); //moves cursor to second line
     i=0;
     while(line2[i] != '\0')
     {
         if (line2[i] != '\0')
             dataWrite(line2[i]);
         i++;
     }
     dataWrite(0b11011111);
     dataWrite('C');


}
//-------------------------------------------------------------------------------------------------------------------
void PortADC_init ()
{
   P5->SEL0 |= BIT4;  // configure pin 5.5 for A0 input
   P5->SEL1 |= BIT4;
   P5->DIR  &= ~BIT4;

}
//------------------------------------------------------------------------------------
void ADC14_init() //Kandalaft code
{
  ADC14 ->CTL0 |= 0b00;   // disable ADC converter during initialization
  ADC14->CTL0  |=      0x04200210;      // S/H pulse mode, SMCLK, 16 sample clocks
  ADC14->CTL1   =      0x00000030;      // 14 bit resolution
  ADC14->CTL1  |=      0x00000000;      // convert for mem0 register
  ADC14->MCTL[0] =     0x00000001;      // ADC14INCHx = 1 for mem[0]
  // ADC14->MCTL[0] =  ADC14->MCTL[0] =  0x00000000;
  // ADC14->CTL0 |=       0x00000002;       // starts the ADC after configuration
  ADC14->CTL0 |=       ADC14_CTL0_ENC;  // enable ADC14ENC, Starts the ADC after confg.
 }
//---------------------------------------------------------------------------------------------------
//reads the temp
void readtemp()
{
    ADC14->CTL0 |= ADC14_CTL0_SC;        //start conversion
    while ( (!ADC14->IFGR0 & BIT0) );     //wait for conversion to complete
    result = ADC14->MEM[0];             // get the value from the ADC
    nADC= ((result*(3300))/16383);   //converts the adc value to voltage in mv
    nADC= (nADC-500)/10;
    sprintf(line2, "%.1f", nADC);

}
