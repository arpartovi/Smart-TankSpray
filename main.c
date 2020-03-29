/*****************************************************
Project : Vehicel motor control
Version : 1.00
Date    : 3/30/2007
Author  : Ali                             
Company : Wega                            

Chip type           : ATmega8
Program type        : Application
Clock frequency     : 16.000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 256
*****************************************************/

#include <mega8.h>
#include <delay.h>
#include <stdio.h>

#define Yled        PORTD.7    
#define Yled_DIR    DDRD.7
#define Gled        PORTD.6   
#define Gled_DIR    DDRD.6
#define ch1_dir     PORTD.4
#define ch1_dir_DIR DDRD.4
#define ch2_dir     PORTD.5
#define ch2_dir_DIR DDRD.5  
#define ch1_out     PORTB1
#define ch1_out_DIR DDRB.1     
#define ch2_out     PORTB2
#define ch2_out_DIR DDRB.2 
#define relay_pin   PORTB.0
#define relay_pin_dir DDRB.0    
#define relay_on()   relay_pin=1
#define relay_off()  relay_pin=0 
#define timer2_on()   TCCR2=0x0A
#define timer2_off()  TCCR2=0x00

#define xtal        16000000
#define baud        57600   
#define ADC_VREF_TYPE 0x40
#define avrage_num  20

 int cycle=0;;
 bit direction=1;;
unsigned int read_adc(unsigned char adc_input)
{     
unsigned char i;
unsigned int sum;
sum=0;
ADMUX=adc_input|ADC_VREF_TYPE; 
for (i=0;i<avrage_num;i++)
 {
     ADCSRA|=0x40;
     while ((ADCSRA & 0x10)==0);
     ADCSRA|=0x10;
     sum=ADCW+sum;
 }    
return sum/avrage_num;
}
    
#define UPE 2
#define OVR 3
#define FE 4
#define UDRE 5
#define RXC 7
#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)
#define FINISH  0
#define BEGIN   1
bit sw_press=FINISH;
void changepwm(unsigned char channel,unsigned char valueH,unsigned char valueL);
void right_direction(unsigned char channel);   
void left_direction(unsigned char channel); 

unsigned char control=0; 

// Timer 2 output compare interrupt service routine
interrupt [TIM2_COMP] void timer2_comp_isr(void)
{  
 static char t=0;       
 t++;
 if (PIND.2==0) t=0;
 if (t==50) {sw_press=FINISH;t=0;timer2_off();}

}

interrupt [EXT_INT0] void ext_int0_isr(void)
{       
char i;    
unsigned int t;

if (sw_press==FINISH)
{
  for (i=1;i<100;i++)
   {
     if (PIND.2!=0) return;
     delay_us(200); 
  }              
  
  if (direction==0)//go backward
   cycle--;
  else 
   cycle++; 
  timer2_on(); 
  sw_press=BEGIN; 
 }  
// delay_ms(10);

}

interrupt [USART_RXC] void uart_rx_isr(void)
{
char status,data;
unsigned char temp,valueL,valueH,pwm_channel,AD_channel,channel;
unsigned int adc_value,smp_num,i; 
char loopL,loopH; ;
Gled=0;
#asm
    push r26
    push r27
    push r30
    push r31
    in   r26,sreg
    push r26
#endasm   
status=UCSRA;
data=UDR;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   control=data; 
else
   goto ret;     
//if control.7&.6:0-->PWM value, 1-->A/D, 2-->Left direction, 3-->Right direction       
temp=control;       
temp=control & 0xc0;  //Mask two hight control's bit
temp=temp>>6;   
switch (temp)
 {
 case 0:
     pwm_channel=control & 0x3f;
     valueH=getchar(); 
     valueL=getchar();
     changepwm(pwm_channel,valueH,valueL); 
     break;
 case 1:   
     AD_channel=control & 0x3f; 
     if (AD_channel==5)//loop number
      {
        loopL=cycle & 0xFF;
        loopH=cycle >>8;
        putchar(loopH);
        putchar(loopL);
      }              
     else  
      {
        adc_value=read_adc(AD_channel); 
        valueL=adc_value & 0xFF;
        valueH=adc_value>>8;
        putchar(valueH);
        putchar(valueL);               
     }   
     break;
 case 2:
     channel=control & 0x3f;
     if(channel==3) 
       relay_on();
     else 
       {
        right_direction(channel);
        direction=0;//go backward
       }
     break;
 case 3:
     channel=control & 0x3f;
     if(channel==3) 
       relay_off();
     else
      { 
        left_direction(channel);
        direction=1;//go forward
      }  
     break; 

 }  
//pwm_value=valueH; //for shifting variable should be 16 bit
//pwm_value= valueL | (pwm_value<<8);    
 
ret:
#asm
    pop  r26
    out  sreg,r26
    pop  r31
    pop  r30
    pop  r27
    pop  r26
#endasm     
}
#pragma savereg+

void main(void)
{        
unsigned int UBRR;
unsigned char a;
Gled_DIR=1;
Yled_DIR=1;           
DDRD.2=0;
PORTD.2=1;
ch1_dir_DIR=1;
ch2_dir_DIR=1; 
ch1_out_DIR=1;
ch2_out_DIR=1;
relay_pin_dir=1;

Gled=1;Yled=1; 
delay_ms(500);
Gled=0;Yled=0; 
// ADC initialization
// ADC Clock frequency: 125.000 kHz
// ADC Voltage Reference: Int., cap. on AREF
ADMUX=ADC_VREF_TYPE;
ADCSRA=0x87;   

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART Mode: Asynchronous
UCSRA=0x00;
UCSRB=0x98;
UCSRC=0x86;
UBRR=xtal/(16*baud)-1;
UBRRL=UBRR;
UBRRH=UBRR>>8;     

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer 1 Stopped
// Mode: Fast PWM top=03FFh
// OC1A output: Non-Inv.
// OC1B output: Non-Inv.
// Noise Canceler: Off
// Input Capture on Falling Edge
TCCR1A=0xA3;
TCCR1B=0x09;
TCNT1H=0x00;
TCNT1L=0x00;

OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;


// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=0x80;
// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Falling Edge
// INT1: Off
GICR|=0x40;
MCUCR=0x02;
//MCUCR=0x00; //low level
GIFR=0x40;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 2000,000 kHz
// Mode: CTC top=OCR2
// OC2 output: Disconnected
ASSR=0x00;
TCCR2=0x0A;
TCNT2=0x00;
OCR2=0xC8;

#asm("sei");
while (1)
      {
        Gled=1;
       // Yled=0;
        delay_ms(300);
        Gled=0;
       // Yled=1;
        delay_ms(300);
      };
}  



void changepwm(unsigned char channel,unsigned char valueH,unsigned char valueL)
{  
  switch (channel)
    { 
     case 1 : OCR1AH=valueH; OCR1AL=valueL; break;
     case 2 : OCR1BH=valueH; OCR1BL=valueL; break;  
    }
    
 }
 
void right_direction(unsigned char channel)
{   

 switch (channel)
    {
     case 1:  ch1_dir=1; break;  //control=0x81
     case 2:  ch2_dir=1; break;  //control=0x82
    } 
}
   
void left_direction(unsigned char channel)
{   
 switch (channel)
   {
     case 1:  ch1_dir=0; break; //control=0xC1
     case 2:  ch2_dir=0; break; //control=0xC2
   } 
}




