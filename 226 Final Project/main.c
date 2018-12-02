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
#include <string.h>

void ADC14_init(void); // initialize ADC
void PortADC_init(void); //initialize ADC14 port
void buttoninit(); //set the buttons
void LEDinit(); //set the lights and LCD pwm pin up
void LCD_init(void);  //initializes the LCD
void RTC_Init();
void speakerinit(); //sets up timerA on 2.4 for the speaker
void timerAinterrupt_init(); //for the wake up lights
void displayinit(); //sets up the screen so we can update what we need

void readtemp(); //function for reading the temp in F
void readpwm(); //function for LDC pwm
void timerA_lights();   //controls lights using timerA
void timedisplay(); //prints the time
void tempdisplay(); //prints the time

//Functions For LCD
void systick_start(void); //prototype for initializing timer
void delay_ms(unsigned); //function prototype for delaying for x ms

void timedisplay(); //prints the time, is passed nADC

void delay_microsec(unsigned microsec);
void PulseEnablePin(void); //sequences the enable pin
void pushNibble(uint8_t nibble);  //puts one nibble onto data pins
void pushByte(uint8_t byte);    //pushes most significant 4 bits to data pins with the push nibble function
void write_command( uint8_t command); //writes one bit of command by calling pushByte() with the command parameter
void dataWrite(uint8_t data);   //will write one bit of data by calling pushByte()


int time_update = 0, alarm_update = 0;
uint8_t hours, mins, secs;
char time[50];


enum states {
    SETTIME,
    ALARM,
    STANDBY,
    SETALARM
            };

static volatile uint16_t result; //vars used in temp reading
float nADC, nADC2;
char temperature[]= "75.6";
char XM='A';
int pwmLCD=0, wakeup=0;
int realtimestatus=1, fasttimestatus=0, settimestatus=0, setalarmstatus=0, onoffstatus=0, snoozestatus=0;
//------------------------------------------------------------------------------------------------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//------------------------------------------------------------------------------------------------------------
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    systick_start(); //Start up Systick

    LCD_init(); // initializations
    RTC_Init();
    ADC14_init();
    PortADC_init();
    buttoninit();
    LEDinit();
    timerAinterrupt_init(); //for the wake up lights
    speakerinit();
    displayinit();

    __enable_interrupt();

    enum states state= STANDBY;

    while(1)
    {
        while(1)
                {
                    switch(state)
                {
                case STANDBY:
                   if(time_update){                            // Time Update Occurred (from interrupt handler)
                        time_update = 0;                        // Reset Time Update Notification Flag
                        sprintf(time,"%02d:%02d:%02d",hours,mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds
                        timedisplay();
                    }

                   if(settimestatus==1)
                       {
                       settimestatus=0;
                       state= SETTIME;
                       }

                   if(setalarmstatus==1)
                   {
                       setalarmstatus=0;
                       state= SETTIME;
                   }

                    break;

                case ALARM:

                    break;

                case SETTIME:
                    write_command(0b10000011); //moves cursor to first line hours position
                    write_command(0b00001101); //changed this to start the blinking cursor


                    break;

                case SETALARM:

                     break;
                default:
                    state= STANDBY;
                }

                }

    }

}

//---------------------------------------------------------------------------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//----------------------------------------------------------------------------------------
//FUCTIONS
//-----------------------------------------------------------------------------------------
void buttoninit() //set up all the buttons used.
//set time 1.0
// realtime 1.1
//fast time 1.4
//set alarm 1.7
//on/off/up 1.6
// snooze/down 1.5
{
    P1->SEL0 &= ~BIT1;  //Onboard Button 1.1 real time
    P1-> SEL1 &= ~BIT1;
    P1-> DIR &= ~BIT1; //input
    P1-> REN |= BIT1;  //enable resistor
    P1-> OUT |= BIT1;  //set input
    P1-> IE |= BIT1; //set an interrupt


    P1->SEL0 &= ~BIT4;  //Onboard Button 1.4 fast time
    P1->SEL1 &= ~BIT4;
    P1->DIR &= ~BIT4;
    P1-> REN |= BIT4;  //enable resistor
    P1-> OUT |= BIT4;  //set input
    P1-> IE |= BIT4; //set an interrupt

    P1->SEL0 &= ~BIT0;  // Button 1.0 set time
    P1->SEL1 &= ~BIT0;
    P1->DIR &= ~BIT0;
    P1-> REN |= BIT0;  //enable resistor
    P1-> OUT |= BIT0;  //set input
    P1-> IE |= BIT0; //set an interrupt

    P1->SEL0 &= ~BIT5;  //Button 1.5 snooze/down
    P1->SEL1 &= ~BIT5;
    P1->DIR &= ~BIT5;
    P1-> REN |= BIT5;  //enable resistor
    P1-> OUT |= BIT5;  //set input
    P1-> IE |= BIT5; //set an interrupt

    P1->SEL0 &= ~BIT6;  //Button 1.6 on/off/up
    P1->SEL1 &= ~BIT6;
    P1->DIR &= ~BIT6;
    P1-> REN |= BIT6;  //enable resistor
    P1-> OUT |= BIT6;  //set input
    P1-> IE |= BIT6; //set an interrupt

    P1->SEL0 &= ~BIT7;  //Button 1.7 set alarm
    P1->SEL1 &= ~BIT7;
    P1->DIR &= ~BIT7;
    P1-> REN |= BIT7;  //enable resistor
    P1-> OUT |= BIT7;  //set input
    P1-> IE |= BIT7; //set an interrupt
}
//--------------------------------------------------------------------------------------
void LEDinit() //P7.7, 7.6, 7.5
{
    P7->SEL0 &= ~BIT5;  //LCD LED 7.5
    P7->SEL1 &= ~BIT5;
    P7->DIR |= BIT5;    //output
    P7-> OUT &= ~BIT5;  //set 0

    P7->SEL0 &= ~BIT6;  //WHITE LED 7.6
    P7->SEL1 &= ~BIT6;
    P7->DIR |= BIT6;    //output
    P7-> OUT &= ~BIT6;  //set 0

    P7->SEL0 &= ~BIT7;  //BLUE LED 7.7
    P7->SEL1 &= ~BIT7;
    P7->DIR |= BIT7;    //output
    P7-> OUT &= ~BIT7;  //set 0

    TIMER_A1->CCR[0]  = 24000;        // PWM Period (# cycles of clock)
    TIMER_A1->CCTL[1] = 0b11100000;     // CCR1 reset/set mode 7
    TIMER_A1->CCTL[2] = 0b11100000;     // CCR1 reset/set mode 7
    TIMER_A1->CCTL[3] = 0b11100000;     // CCR1 reset/set mode 7
    TIMER_A1->CTL = 0b1001010000;


}
//------------------------------------------------------------------------------------------
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
    write_command(0b00001100); //changed this to stop the blinking cursor
    delay_microsec(100);
    write_command(1);
    delay_microsec(100);
    write_command(6);
    delay_ms(10);
}
//-------------------------------------------------------------------------------------------------------------------
void PortADC_init ()
{
   P5->SEL0 |= BIT5;  // configure pin 5.5 for A0 input A0.0
   P5->SEL1 |= BIT5;
   P5->DIR  &= ~BIT5;

   P5->SEL0 |= BIT4;  // configure pin 5.4 for A0 input A0.1
   P5->SEL1 |= BIT4;   //LCD brightness control
   P5->DIR  &= ~BIT4;

}
//------------------------------------------------------------------------------------
void ADC14_init() //Kandalaft code
{
  ADC14 ->CTL0 |= 0b00;   // disable ADC converter during initialization
  ADC14->CTL0  |=      0x04200210;      // S/H pulse mode, SMCLK, 16 sample clocks
  ADC14->CTL1   =      0x00000030;      // 14 bit resolution
  ADC14->CTL1  |=      0x00000000;      // convert for mem0 register

  ADC14->MCTL[0] =     0x00000000;      // ADC14INCHx = 1 for mem[0] TEMPERATURE
  ADC14->MCTL[1] =     0x00000001;      // ADC14INCHx = 1 for mem[1] LCD BRIGHTNESS

  ADC14->CTL0 |=       ADC14_CTL0_ENC;  // enable ADC14ENC, Starts the ADC after confg.


 }
//---------------------------------------------------------------------------------------------------
void RTC_Init(){
    //Initialize time to 12:00:00 am

    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = 59<<8 | 55;//0 min, 0 secs
    RTC_C->TIM1 = 0<<8 | 0;  //sunday, 12 pm
    RTC_C->YEAR = 2018;
    //Alarm at 2:46 pm
    RTC_C->AMINHR = 14<<8 | 46 | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
    RTC_C->ADOWDAY = 0;
    RTC_C->PS1CTL = 0b11010;  //1/64 second interrupt CHECK THIS

    RTC_C->CTL0 = (0xA500) | BIT5; //turn on interrupt
    RTC_C->CTL13 = 0;

    NVIC_EnableIRQ(RTC_C_IRQn);
}
//--------------------------------------------------------------------------------------------------------
void speakerinit()
{
P2->SEL0 |= BIT4;
P2->SEL1 &= ~(BIT4);
P2->DIR |= BIT4;  // P2.4 set TA0.1



 TIMER_A0->CCR[0]  = 37500-1;              // PWM Period (# cycles of clock)
 TIMER_A0->CCTL[1] = 0b11100000;     // CCR1 reset/set mode 7
 TIMER_A0->CCR[1]  = 37500-2; // CCR1 PWM duty cycle

 TIMER_A0->CTL = 0b1000010000; //smclk, stop mode, no divder,

}
//-----------------------------------------------------------------------------------
void timerAinterrupt_init() //for the wake up lights
{
    TIMER_A0->CCR[0]  = 30000-1;              // PWM Period (# cycles of clock)

    TIMER_A0->CTL = 0b1000000010; //smclk, stop mode, no divder, IE enabled  dont want it to be on untill 5 min before alarm
   // TIMER_A0->CTL = 0b1000010010; //smclk, up mode, no divder, IE enabled

    NVIC_EnableIRQ(TA2_0_IRQn);

}
//---------------------------------------------------------------------------------------
void displayinit()
{
    write_command(0b00000001); //reset display
    char line1[]= "   12:00:00 AM";
    char line2[]= "Alarm:";
    char line3[]= "OFF      Temp:";
    char line4[]= "00:00AM  00.0";


   int i=0;
   while(line1[i] != '\0')
   {
       if (line1[i] != '\0')
           dataWrite(line1[i]);
       i++;
   }
   write_command(0b11000000); //moves cursor to the second line
   i=0;
   while(line2[i] != '\0')
   {
       if (line2[i] != '\0')
           dataWrite(line2[i]);
       i++;
   }
   write_command(0b10010000); //moves cursor to the third line
   i=0;
   while(line3[i] != '\0')
   {
       if (line3[i] != '\0')
           dataWrite(line3[i]);
       i++;
   }
   write_command(0b11010000); //moves cursor to the fourth line
   i=0;
   while(line4[i] != '\0')
   {
       if (line4[i] != '\0')
           dataWrite(line4[i]);
       i++;
   }
   dataWrite(0b11011111); //degree symbol
   dataWrite('F');
}

//--------------------------------------------------------------------------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//---------------------------------------------------------------------------------------

void timedisplay()//prints the current time
{
    if(hours==0)
       {
        XM='A';
        sprintf(time,"12:%02d:%02d",mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds
       }
    if(hours<=12)
     {
         if(hours<=9)
            {
             sprintf(time," %d:%02d:%02d",hours,mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds
             XM='A';
            }
         if(hours>9)
             {
             XM='A';
             sprintf(time,"%02d:%02d:%02d",hours,mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds
             if(hours==12)
                 XM= 'P';
             }
     }

    if((hours>12))
    {    XM= 'P';
        if(hours<=21)
        {
            hours=hours-12;
            sprintf(time," %d:%02d:%02d",hours,mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds

        }
        if(hours>21)
        {
            hours=hours-12;
            sprintf(time,"%02d:%02d:%02d",hours,mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds

        }
    }
    int i=0;
    write_command(0b10000011); //moves cursor to first line hours position
    while(time[i] != '\0')
    {
        if (time[i] != '\0')
            dataWrite(time[i]);
        i++;
    }
    dataWrite(' ');
    dataWrite(XM);
    tempdisplay();
}
//--------------------------------------------------------------------------------------------------------------------
void tempdisplay()//prints the temperature, is passed nADC
{
    readtemp();
     int i=0;
     write_command(0b11011001); //moves cursor to forth line 10th spot
     while(temperature[i] != '\0')
     {
         if (temperature[i] != '\0')
             dataWrite(temperature[i]);
         i++;
     }
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
    nADC= (9.0/5.0)*nADC + 32; //to degrees F
    sprintf(temperature, "%.1f", nADC);

}
//---------------------------------------------------------------------------------------------------
//reads the LCD pwm
void readpwm()
{
    ADC14->CTL0 |= ADC14_CTL0_SC;        //start conversion
    while ( (!ADC14->IFGR0 & BIT0) );     //wait for conversion to complete
    result = ADC14->MEM[1];             // get the value from the ADC
    nADC2= ((result*(3300))/16383);   //converts the adc value to voltage in mv
    nADC2= (nADC2/330.0) *100; //should be int between 0 and 100
    pwmLCD= nADC2;

}

//-------------------------------------------------------------------------------------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//---------------------------------------------------------------------------------------------------
void RTC_C_IRQHandler()
{
    if(RTC_C->PS1CTL & BIT0){                           // PS1 Interrupt Happened
        hours = RTC_C->TIM1 & 0x00FF;                   // Record hours (from bottom 8 bits of TIM1)
        mins = (RTC_C->TIM0 & 0xFF00) >> 8;             // Record minutes (from top 8 bits of TIM0)
        secs = RTC_C->TIM0 & 0x00FF;                    // Record seconds (from bottom 8 bits of TIM0)
        // For increasing the number of seconds  every PS1 interrupt (to allow time travel)
//        if(secs != 59){                                 // If not  59 seconds, add 1 (otherwise 59+1 = 60 which doesn't work)
//            RTC_C->TIM0 = RTC_C->TIM0 + 1;
//        }
//        else {
//            RTC_C->TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8)+1)<<8;  // Add a minute if at 59 seconds.  This also resets seconds.
//                                                                 // TODO: What happens if minutes are at 59 minutes as well?
//            time_update = 1;                                     // Send flag to main program to notify a time update occurred.
//        }
        time_update = 1;
        RTC_C->PS1CTL &= ~BIT0;                         // Reset interrupt flag
    }
    if(RTC_C->CTL0 & BIT1)                              // Alarm happened!
    {
        alarm_update = 1;                               // Send flag to main program to notify a time update occurred.
        RTC_C->CTL0 = (0xA500) | BIT5;                  // Resetting the alarm flag.  Need to also write the secret code
                                                        // and rewrite the entire register.
                                                        // TODO: It seems like there is a better way to preserve what was already
                                                        // there in case the setup of this register needs to change and this line
                                                        // is forgotten to be updated.
    }
}

//------------------------------------------------------------------------------------------------------------------------
// interrupt handler to set button status when pushed
void PORT1_IRQHandler()
{
    if (P1->IFG &BIT0)
    {
        P1->IFG &= ~BIT0;
        settimestatus=1;
    }
    if (P1->IFG &BIT1)
    {
        P1->IFG &= ~BIT1;
        realtimestatus=1;
    }
    if(P1->IFG &BIT4)
    {
        P1->IFG &= ~BIT4;
        fasttimestatus=1;

    }
    if (P1->IFG &BIT5)
    {
        P1->IFG &= ~BIT5;
        snoozestatus=1;

    }
    if (P1->IFG &BIT6)
    {
        P1->IFG &= ~BIT6;
        onoffstatus=1;
    }
    if (P1->IFG &BIT7)
    {
        P1->IFG &= ~BIT7;
        setalarmstatus=1;

    }

}
//--------------------------------------------------------------------------------------------
//wake up lights
void TA2_0_IRQHandler()
{
    if (TIMER_A2->CTL & BIT0)
    {
        wakeup+=1;
        TIMER_A0-> CCR[1]= (wakeup/100.00)*24000;
        TIMER_A0-> CCR[2]= (wakeup/100.00)*24000;

    }
}
//------------------------------------------------------------------------------------------------------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//--------------------------------------------------------------------------------------------------------------------
//Starts the systick timer for LCD delays
void systick_start() //initialize timer
{
    SysTick-> CTRL = 0; //off
    SysTick-> LOAD = 0xBB8; //1ms+clock
    SysTick-> VAL = 0; //reset count
    SysTick-> CTRL = 5; //enable with interrupt

}

//-----------------------------------------------------------------------------------------
//takes in amount of mili seconds to delay
void delay_ms(unsigned ms) //function for delaying for x ms. takes in ms
{
    SysTick-> LOAD = (3000* ms)-1; //1ms+clock
    SysTick-> VAL = 0; //reset count
    while((SysTick-> CTRL & 0x10000) == 0) {} ;  //function given in lab

}
//------------------------------------------------------------------------------------------
//takes in micro seconds to delay
void delay_microsec(unsigned microsec) //function for delaying for x ms. takes in ms
{
    SysTick-> LOAD = (300* microsec)-1; //1micros+clock
    SysTick-> VAL = 0; //reset count
    while((SysTick-> CTRL & 0x10000) == 0) {} ;  //function given in lab

}

//------------------------------------------------------------------------------------------
void PulseEnablePin(void) //sequences the enable pin- kandalaft code
{
    P6-> OUT &= ~BIT1; //set pulse to 0V
    delay_microsec(10);
    P6-> OUT |= BIT1;
    delay_microsec(10);
    P6-> OUT &= ~BIT1; //set pulse to 0V
    delay_microsec(10);

}

//--------------------------------------------------------------------------------------------
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
