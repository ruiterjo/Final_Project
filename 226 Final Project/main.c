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
void timer32interrupt_init(); //for the wake up lights
void displayinit(); //sets up the screen so we can update what we need

void readtemp(); //function for reading the temp in F
void readpwm(); //function for LDC pwm
void timerA_lights();   //controls lights using timerA
void timedisplay(); //prints the time
void timedisplaySET();//prints the current time that you are trying to set
void alarmdisplay();
void alarmdisplaysnooze();
void tempdisplay(); //prints the time
void sethoursmins(); //writes to rtc
void setalarm();
uint8_t debounceSETTIME(); //function prototype for debouncing a switch
uint8_t debounceSETALARM(); //function prototype for debouncing a switch
uint8_t debounceDOWN(); //function prototype for debouncing a switch
uint8_t debounceUP(); //function prototype for debouncing a switch

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

//-----------------------------------------------------------------------------------------------------------

int time_update = 0, alarm_update = 0;
uint8_t hours, mins, secs;
uint8_t sethours=0, setmins=0;
uint8_t setAhours=0, setAmins=0;
uint8_t snoozemins=0;
char time[50];
char timeSET[50];
char alarmSET[50];


enum states {
    SETTIMEH,
    SETTIMEM,
    ALARM,
    STANDBY,
    SETALARMH,
    SETALARMM
};

static volatile uint16_t result; //vars used in temp reading
float nADC, nADC2, pwmLCD;
char temperature[]= "75.6";
char XM='A';
int wakeup=0;
int realtimestatus=1, fasttimestatus=0, settimestatus=0,
        setalarmstatus=0, onoffstatus1=0, onoffstatus=0, snoozestatus=0, alarmstatus=0;
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
    timer32interrupt_init(); //for the wake up lights
    speakerinit();
    displayinit();



    NVIC_EnableIRQ(RTC_C_IRQn);
    //NVIC_EnableIRQ(PORT1_IRQn);

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
                   TIMER_A0->CCR[1] = 0;


                   if(debounceSETTIME())
                       {
                       state= SETTIMEH;
                       }

                   if(debounceSETALARM())
                   {
                       state= SETALARMH;
                   }
                   if(alarm_update)
                   {
                       state= ALARM;
                   }
                   if((hours==setAhours) && ((setAmins-mins)==5) && (alarmstatus))
                   {
                       TIMER32_1->CONTROL  = 0b11100010; //enable the interrupt
                   }

                   if(debounceUP())
                   {
                       onoffstatus1+=1;
                       if(onoffstatus1==1)
                       {
                           write_command(0b10010000); //moves cursor to the third line
                           dataWrite('O');
                           dataWrite('N');
                           dataWrite(' ');
                           dataWrite(' ');
                           dataWrite(' ');
                           dataWrite(' ');
                           //enable alarm
                           RTC_C->CTL0 = ((0xA500) | BIT5);
                           RTC_C->AMINHR = setAhours<<8 | setAmins | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
                           TIMER_A0->CCR[1] = 0;
                           alarmdisplay();
                           alarmstatus=1;

                       }
                       if(onoffstatus1==2)
                       {
                           write_command(0b10010000); //moves cursor to the third line
                           dataWrite('O');
                           dataWrite('F');
                           dataWrite('F');
                           dataWrite(' ');
                           dataWrite(' ');
                           dataWrite(' ');
                           //disable alarm
                           RTC_C->CTL0 = ((0xA500));
                           RTC_C->AMINHR = setAhours<<8 | setAmins;  //bit 15 and 7 are Alarm Enable bits
                           onoffstatus1=0;
                           alarmdisplay();
                           TIMER_A0->CCR[1] = 0;
                           alarmstatus=0;


                       }

                   }


                    break;
//-----------------------------------------------------------------------------------------
                case ALARM:
                    alarm_update=0; //reset flag
                    TIMER_A1->CCR[2] = 50000;
                    TIMER_A1->CCR[1] = 50000;
                    TIMER32_1->CONTROL  = 0b11000010; //enable the interrupt
                    wakeup=0;
                    int sound=1;
                    P1->IFG &= ~BIT6;
                    P1->IFG &= ~BIT5; //clear flags just in case
                 while(sound)
                 {
                    if(time_update){                            // Time Update Occurred (from interrupt handler)
                         time_update = 0;                        // Reset Time Update Notification Flag
                         sprintf(time,"%02d:%02d:%02d",hours,mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds
                         timedisplay();
                    }

                    TIMER_A0->CCR[1] = 1500000 / 294;
                    delay_ms(100);
                    TIMER_A0->CCR[1] = 0;
                    delay_ms(100);


                    if (P1->IFG &BIT5)
                    {
                        P1->IFG &= ~BIT5;   //clear flag

                        TIMER_A0->CCR[1] = 0;
                        write_command(0b10010000); //moves cursor to the third line
                        TIMER_A0->CCR[1] = 0;
                        dataWrite('S');
                        dataWrite('N');
                        dataWrite('O');
                        dataWrite('O');
                        dataWrite('Z');
                        dataWrite('E');
                        //change alarm
                        snoozemins= mins+10;
                        RTC_C->CTL0 = ((0xA500) | BIT5);
                        RTC_C->AMINHR =  setAhours<<8 | snoozemins | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
                        alarmdisplaysnooze();
                        snoozestatus=0;
                        sound=0;
                        TIMER_A1->CCR[2] = 0;
                        TIMER_A1->CCR[1] = 0;
                        wakeup=0; //resets lights

                    }

                    if (P1->IFG &BIT6)
                       {
                         P1->IFG &= ~BIT6;   //clear flag
                         sound=0;
                         TIMER_A0->CCR[1] = 0;
                         setalarm(); //sets it back to previous time
                         TIMER_A1->CCR[2] = 0;
                         TIMER_A1->CCR[1] = 0;
                         write_command(0b10010000); //moves cursor to the third line
                        dataWrite('O');
                        dataWrite('N');
                        dataWrite(' ');
                        dataWrite(' ');
                        dataWrite(' ');
                        dataWrite(' ');
                        alarmdisplay();
                        wakeup=0; //resets lights

                         }
                 }


              state=STANDBY;
              onoffstatus=0;


                    break;
//------------------------------------------------------------------------------------------

                case SETTIMEH:
                    settimestatus= 1;
                    while((settimestatus))
                    {    write_command(0b10000011); //moves cursor to first line hours position
                    delay_ms(70);
                    dataWrite(' ');
                    dataWrite(' ');


                        if (P1->IFG & BIT6)
                      {
                         P1->IFG &= ~BIT6;
                        if(sethours<=23)
                        {
                            sethours+=1;
                        }
                        else
                        {
                            sethours=0;
                        }
                      }
                        if (P1->IFG & BIT5)
                      {
                         P1->IFG &= ~BIT5;
                        if(sethours>=1)
                        {
                            sethours-=1;
                        }
                        else
                        {
                            sethours=23;
                        }
                    }
                    sethoursmins();
                    timedisplaySET();
                    if( debounceSETTIME())
                        settimestatus=0;
                }
                    if(settimestatus=1)
                        {
                        hours=sethours;
                        state= SETTIMEM;
                        }

                    break;
 //--------------------------------------------------------------------------------
                case SETTIMEM:
                    settimestatus=1;
                    while((settimestatus))
                    {    write_command(0b10000110); //moves cursor to first line minutes position
                    delay_ms(70);
                    dataWrite(' ');
                    dataWrite(' ');


                        if (P1->IFG & BIT6)
                      {
                         P1->IFG &= ~BIT6;
                        if(setmins<59)
                        {
                            setmins+=1;
                        }
                        else
                        {
                            setmins=0;
                        }
                    }
                        if (P1->IFG & BIT5)
                      {
                         P1->IFG &= ~BIT5;
                        if(setmins>0)
                        {
                            setmins-=1;
                        }
                        else
                        {
                            setmins=59;
                        }
                    }
                    sethoursmins();
                    timedisplaySET();
                    if( debounceSETTIME())
                        settimestatus=0;
                    }
                    if(settimestatus=1)
                        {
                        mins=setmins;
                        state= STANDBY;
                        }

                    break;
//-------------------------------------------------------------------------------
                case SETALARMH:
                    setalarmstatus= 1;
                    while((setalarmstatus))
                    {  write_command(0b11010000); //moves cursor to the fourth line hours
                    delay_ms(70);
                    dataWrite(' ');
                    dataWrite(' ');

                        if (P1->IFG & BIT6)
                      {
                         P1->IFG &= ~BIT6;
                        if(setAhours<=23)
                        {
                            setAhours+=1;
                        }
                        else
                        {
                            setAhours=0;
                        }
                    }
                        if (P1->IFG & BIT5)
                      {
                         P1->IFG &= ~BIT5;
                        if(setAhours>=1)
                        {
                            setAhours-=1;
                        }
                        else
                        {
                            setAhours=23;
                        }
                    }
                    setalarm();
                    alarmdisplay();
                    if( debounceSETALARM())
                        setalarmstatus=0;
                }
                    if(setalarmstatus=1)
                        state= SETALARMM;
                     break;
//---------------------------------------------------------------------------------------
                case SETALARMM:

                    setalarmstatus=1;
                    while((setalarmstatus))
                     {  write_command(0b11010011); //moves cursor to the fourth line minutes
                     delay_ms(70);
                     dataWrite(' ');
                     dataWrite(' ');

                        if (P1->IFG & BIT6)
                      {
                         P1->IFG &= ~BIT6;
                        if(setAmins<59)
                        {
                            setAmins+=1;
                        }
                        else
                        {
                            setAmins=0;
                        }
                    }
                        if (P1->IFG & BIT5)
                      {
                         P1->IFG &= ~BIT5;
                        if(setAmins>0)
                        {
                            setAmins-=1;
                        }
                        else
                        {
                            setAmins=59;
                        }
                    }
                    setalarm();
                    alarmdisplay();
                    if( debounceSETALARM())
                        setalarmstatus=0;
                    }
                    if(setalarmstatus=1)
                        state= STANDBY;

                     break;
//-------------------------------------------------------------------------------------------------
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
    P7->SEL0 |=BIT5;  //LCD LED 7.5 TA1.3
    P7->SEL1 &= ~BIT5;
    P7->DIR |= BIT5;    //output


    P7->SEL0 |= BIT6;  //WHITE LED 7.6 TA1.2
    P7->SEL1 &= ~BIT6;
    P7->DIR |= BIT6;    //output


    P7->SEL0 |= BIT7;  //BLUE LED 7.7 TA1.1
    P7->SEL1 &= ~BIT7;
    P7->DIR |= BIT7;    //output


    TIMER_A1->CCR[0]  = 50000;        // PWM Period (# cycles of clock)
    TIMER_A1->CCTL[1] = 0b11100000;     // CCR1 reset/set mode 7
    TIMER_A1->CCR[1]  = 0;
    TIMER_A1->CCTL[2] = 0b11100000;     // CCR1 reset/set mode 7
    TIMER_A1->CCR[2]  = 0;
    TIMER_A1->CCTL[3] = 0b11100000;     // CCR1 reset/set mode 7
    TIMER_A1->CCR[3]  = 0;
    TIMER_A1->CTL = 0b1000010000;


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

   P5->SEL0 |= BIT2;  // configure pin 5.2 for A3 input A0.1
   P5->SEL1 |= BIT2;   //LCD brightness control
   P5->DIR  &= ~BIT2;

}
//------------------------------------------------------------------------------------
void ADC14_init() //Kandalaft code
{
  ADC14-> CTL0 &= ~ADC14_CTL0_ENC;
  ADC14->CTL0  |=      0X04270210;      // S/H pulse mode, SMCLK, 16 sample clocks
  ADC14->CTL1   =      0x00000030;      // 14 bit resolution


  ADC14->MCTL[0] =     0;      // ADC14INCHx = 0 for mem[0] TEMPERATURE
  ADC14->MCTL[1] =     3;      // ADC14INCHx = 1 for mem[1] LCD BRIGHTNESS

  ADC14->CTL0 |=       ADC14_CTL0_ENC;  // enable ADC14ENC, Starts the ADC after confg.


 }
//---------------------------------------------------------------------------------------------------
void RTC_Init(){
    //Initialize time to 12:00:00 am

    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = 59<<8 | 55;//0 min, 0 secs
    RTC_C->TIM1 =  0;  // 12 am
    //Alarm at 2:46 pm
    RTC_C->AMINHR = 0<<8 | 0 ;  //bit 15 and 7 are Alarm Enable bits are not enabled
    RTC_C->ADOWDAY = 0;
    RTC_C->PS1CTL = 0b11010;

    RTC_C->CTL0 = ((0xA500) | BIT5); //turn on interrupt




}
//--------------------------------------------------------------------------------------------------------
void speakerinit()
{
P2->SEL0 |= BIT4;
P2->SEL1 &= ~(BIT4);
P2->DIR |= BIT4;  // P2.4 set TA0.1


TIMER_A0->CCR[0] = 3000000 / 294;  //Math in an interrupt is bad behavior, but shows how things are happening.  This takes our clock and divides by the frequency of this note to get the period.

 TIMER_A0->CCTL[1] = 0b11100000;     // CCR1 reset/set mode 7
 TIMER_A0->CCR[1]  = 0; // CCR1 PWM duty cycle

 TIMER_A0->CTL = 0b1000010000; //smclk, UP mode, no divder,

}
//-----------------------------------------------------------------------------------
void timer32interrupt_init() //for the wake up lights
{
    TIMER32_1->CONTROL  = 0b11000010;              // periodic, wrapping, bit 5 is enable interrupt, 32bit
    TIMER32_1-> LOAD= 9000000;

    NVIC_EnableIRQ(T32_INT1_IRQn);

}
//---------------------------------------------------------------------------------------
void displayinit()
{
    write_command(0b00000001); //reset display
    char line1[]= "   12:00:00 AM";
    char line2[]= "Alarm:";
    char line3[]= "---      Temp:";
    char line4[]= "--:--AM  --.-";


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

    if(hours<=12)
     {
        if(hours==0)
           {
            XM='A';
            sprintf(time,"12:%02d:%02d",mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds
           }
        if((hours<=9)&& (hours>=1))
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
    readpwm();
    tempdisplay();

}
//---------------------------------------------------------------------------------------------------------------------
void timedisplaySET()//prints the current time that you are trying to set
{

    if(sethours<=12)
     {
        if(sethours==0)
           {
            XM='A';
            sprintf(timeSET,"12:%02d:00",setmins); // Print time with mandatory 2 digits  each for hours, mins, seconds
           }
        if((sethours<=9)&& (sethours>=1))
            {
             sprintf(timeSET," %d:%02d:00",sethours,setmins); // Print time with mandatory 2 digits  each for hours, mins, seconds
             XM='A';
            }
         if(sethours>9)
             {
             XM='A';
             sprintf(timeSET,"%02d:%02d:00",sethours,setmins); // Print time with mandatory 2 digits  each for hours, mins, seconds
             if(sethours==12)
                 XM= 'P';
             }
     }

    if((sethours>12))
    {    XM= 'P';
        if(sethours<=21)
        {
            sprintf(timeSET," %d:%02d:00",(sethours-12),setmins); // Print time with mandatory 2 digits  each for hours, mins, seconds

        }
        if(sethours>21)
        {
            sprintf(timeSET,"%02d:%02d:00",(sethours-12),setmins); // Print time with mandatory 2 digits  each for hours, mins, seconds

        }
    }
    int i=0;
    write_command(0b10000011); //moves cursor to first line hours position
    while(timeSET[i] != '\0')
    {
        if (timeSET[i] != '\0')
            dataWrite(timeSET[i]);
        i++;
    }
    dataWrite(' ');
    dataWrite(XM);
}
//-------------------------------------------------------------------------------------------------------------------
void alarmdisplay()
{
    {

        if(setAhours<=12)
         {
            if(setAhours==0)
               {
                XM='A';
                sprintf(alarmSET,"12:%02d",setAmins); // Print time with mandatory 2 digits  each for hours, mins, seconds
               }
            if((setAhours<=9)&& (setAhours>=1))
                {
                 sprintf(alarmSET," %d:%02d",setAhours,setAmins); // Print time with mandatory 2 digits  each for hours, mins, seconds
                 XM='A';
                }
             if(setAhours>9)
                 {
                 XM='A';
                 sprintf(alarmSET,"%02d:%02d",setAhours,setAmins); // Print time with mandatory 2 digits  each for hours, mins, seconds
                 if(setAhours==12)
                     XM= 'P';
                 }
         }

        if((setAhours>12))
        {    XM= 'P';
            if(setAhours<=21)
            {
                sprintf(alarmSET," %d:%02d",(setAhours-12),setAmins); // Print time with mandatory 2 digits  each for hours, mins, seconds

            }
            if(setAhours>21)
            {
                sprintf(alarmSET,"%02d:%02d",(setAhours-12),setAmins); // Print time with mandatory 2 digits  each for hours, mins, seconds

            }
        }
        int i=0;
        write_command(0b11010000); //moves cursor to fourth line
        while(alarmSET[i] != '\0')
        {
            if (alarmSET[i] != '\0')
                dataWrite(alarmSET[i]);
            i++;
        }
        dataWrite(XM);
    }
}
//--------------------------------------------------------------------------------------------------------------------
void alarmdisplaysnooze()
{
    {

        if(setAhours<=12)
         {
            if(setAhours==0)
               {
                XM='A';
                sprintf(alarmSET,"12:%02d",snoozemins); // Print time with mandatory 2 digits  each for hours, mins, seconds
               }
            if((setAhours<=9)&& (setAhours>=1))
                {
                 sprintf(alarmSET," %d:%02d",setAhours,snoozemins); // Print time with mandatory 2 digits  each for hours, mins, seconds
                 XM='A';
                }
             if(setAhours>9)
                 {
                 XM='A';
                 sprintf(alarmSET,"%02d:%02d",setAhours,snoozemins); // Print time with mandatory 2 digits  each for hours, mins, seconds
                 if(setAhours==12)
                     XM= 'P';
                 }
         }

        if((setAhours>12))
        {    XM= 'P';
            if(setAhours<=21)
            {
                sprintf(alarmSET," %d:%02d",(setAhours-12),snoozemins); // Print time with mandatory 2 digits  each for hours, mins, seconds

            }
            if(setAhours>21)
            {
                sprintf(alarmSET,"%02d:%02d",(setAhours-12),snoozemins); // Print time with mandatory 2 digits  each for hours, mins, seconds

            }
        }
        int i=0;
        write_command(0b11010000); //moves cursor to fourth line
        while(alarmSET[i] != '\0')
        {
            if (alarmSET[i] != '\0')
                dataWrite(alarmSET[i]);
            i++;
        }
        dataWrite(XM);
    }
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
     dataWrite(0b11011111); //degree symbol
     dataWrite('F');
     dataWrite(' ');
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
    while ( (!ADC14->IFGR0 & BIT1) );     //wait for conversion to complete
    result = ADC14->MEM[1];             // get the value from the ADC
    nADC2= ((result*(3300))/16383);   //converts the adc value to voltage in mv
    nADC2= (nADC2/3300.0); //should be int between 0 and 100
    pwmLCD= nADC2 ;




    TIMER_A1->CCR[3]  =50000*(pwmLCD);        // PWM Period (# cycles of clock)


    ADC14->CTL1   &=~  2;      // 14 bit resolution
    ADC14->CTL1  |=    2;


}
void sethoursmins()
{
    //RTC_C->CTL0 = (0xA500);

    RTC_C->TIM0 = setmins<<8;//x min, 0 secs
    RTC_C->TIM1 =  sethours;  // reads from hours

    //RTC_C->CTL13 = 0;

}
//--------------------------------------------------------------------------------------------------
void setalarm()
{
    RTC_C->CTL0 = ((0xA500));

    RTC_C->AMINHR = setAhours<<8 | setAmins |BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits not enabled

    RTC_C->CTL0 = ((0xA500) | BIT5);
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

        time_update = 1;
        RTC_C->PS1CTL &= ~BIT0;                         // Reset interrupt flag
    }
    if(RTC_C->CTL0 & BIT1)                              // Alarm happened!
    {
        alarm_update = 1;                               // Send flag to main program to notify a time update occurred.
        RTC_C->CTL0 = (0xA500);                  // Resetting the alarm flag.  Need to also write the secret code
                                                        // and rewrite the entire register.

    }
}

//------------------------------------------------------------------------------------------------------------------------
// interrupt handler to set button status when pushed
void PORT1_IRQHandler()
{

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

    P1->IFG &= ~BIT0;
    P1->IFG &= ~BIT1;
    P1->IFG &= ~BIT4;
    P1->IFG &= ~BIT5;
    P1->IFG &= ~BIT6;
    P1->IFG &= ~BIT7;


}
//--------------------------------------------------------------------------------------------
//wake up lights
void T32_INT1_IRQHandler()
{
        TIMER32_1->INTCLR=1;

        if(wakeup<100)
            wakeup+=1;

        TIMER_A1-> CCR[1]= (wakeup/100.0)*50000;
        TIMER_A1-> CCR[2]= (wakeup/100.0)*50000;


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
    while((SysTick-> CTRL & 0x00010000) == 0) {} ;  //function given in lab

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
//-----------------------------------------------------------------------------------------------------------------
uint8_t debounceSETTIME(void)   //this section of code is based on lab prep from Zuidema
{
    uint8_t pin_val= 0; //var set to low
    if (!(P1->IN &BIT0)) //check button status
        {
        P1->IFG &= ~BIT0; //clear flags
        delay_ms(5); // pause for 5ms for bounce
        if (!(P1->IN &BIT0)) //check if still pushed
            pin_val=1;
        while(!(P1->IN &BIT0)) {};
        }
        return pin_val;  //returns 1 if pushed and 0 if not
 }
//-----------------------------------------------------------------------------------------------------------------
uint8_t debounceSETALARM(void)   //this section of code is based on lab prep from Zuidema
{
    uint8_t pin_val= 0; //var set to low
    if (!(P1->IN &BIT7)) //check button status
        {
        P1->IFG &= ~BIT7; //clear flags
        delay_ms(5); // pause for 5ms for bounce
        if (!(P1->IN &BIT7)) //check if still pushed
            pin_val=1;
        while(!(P1->IN &BIT7)) {};
        }
        return pin_val;  //returns 1 if pushed and 0 if not
 }
//-----------------------------------------------------------------------------------------------------------------
uint8_t debounceDOWN(void)   //this section of code is based on lab prep from Zuidema
{
    uint8_t pin_val= 0; //var set to low
    if (!(P1->IN &BIT5)) //check button status
        {
        P1->IFG &= ~BIT5; //clear flags
        delay_ms(5); // pause for 5ms for bounce
        if (!(P1->IN &BIT5)) //check if still pushed
            pin_val=1;
        while(!(P1->IN &BIT5)) {};
        }
        return pin_val;  //returns 1 if pushed and 0 if not
 }
//-----------------------------------------------------------------------------------------------------------------
uint8_t debounceUP(void)   //this section of code is based on lab prep from Zuidema
{
    uint8_t pin_val= 0; //var set to low
    if (!(P1->IN &BIT6)) //check button status
        {
        P1->IFG &= ~BIT6; //clear flags
        delay_ms(5); // pause for 5ms for bounce
        if (!(P1->IN &BIT6)) //check if still pushed
            pin_val=1;
        while(!(P1->IN &BIT6)) {};
        }
        return pin_val;  //returns 1 if pushed and 0 if not
 }

