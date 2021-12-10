/*
 * AVR128_CAN.c
 *
 * Created: 8/11/2021 1:33:20 PM
 * Author : 
 */ 


#define F_CPU 24000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/xmega.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "MCP_2515_LIB.h"
#include "UART_1_AVR128DA64.h"
#include "SPI_0_AVR128DA64.h"
//struct can_frame canMsg1;
struct can_frame canMsg2;
struct can_frame canMsg3;
struct can_frame canMsg4;
//struct can_frame canMsg;
char send1[8]="";
//struct can_frame canMsg;
void rcv_data();
char buff2[50];
uint8_t rcv_vle=0;
bool can_flag = false;
void rcv_data()
{
	struct can_frame canMsg;			// define data frame to be receive
	char buff1[20];
	char buff2[20];
	
	readmsg(&canMsg);					//read can Msg in canMsg struct variable
	
	// 	USART1_sendInt(canMsg.can_id);
	//
	// 	sprintf(buff1,"%d",canMsg.can_dlc);
	// 	USART1_sendString(buff1);
	
	_delay_ms(20);
	
	for(uint8_t i =0;i<2;i++)
	{
		sprintf(buff2,"%c",canMsg.data[i]);
		//USART1_sendString(buff2);
	}
	
	_delay_ms(100);
	
	//USART1_sendChar(' ');
	// USART1_sendChar('\n');
	
	can_rcv_vle = ((canMsg.data[0]-48)*10)+(canMsg.data[1]-48);
		USART1_sendInt(can_rcv_vle);
		USART1_sendChar('\n');
		 if(can_rcv_vle==70)
		 {
			 PORTC.OUTCLR= PIN6_bm; 
		     _delay_ms(1000);
			  PORTC.OUTSET= PIN6_bm;
			  _delay_ms(500);
			 
		 }
		
}



 static void PORT_init(void)
 {
	 //PORTC.DIR |= PIN7_bm;
	PORTC.DIRCLR = PIN2_bm;
	 PORTC.PIN2CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
	 
 }

 ISR(PORTC_PORT_vect)
 {
	
	can_flag=true;
	 PORTC.INTFLAGS = PIN2_bm;
	// snd_can_status = 1;
// 	  can_send(45);
// 	  rcv_data();
 }
		 
		 
int main(void)
{
	 sei();

   _PROTECTED_WRITE (CLKCTRL.OSCHFCTRLA, ((CLKCTRL_FREQSEL_24M_gc)|(CLKCTRL_AUTOTUNE_bm)));			//To increase clock frq
	USART1_init(9600);
	PORT_init();
	SPI_0_init();
	mcp_init();
    PORTC.DIRSET=PIN6_bm;
// 	PORTC.DIRCLR = PIN2_bm;
// 	PORTC.PIN2CTRL |= PORT_PULLUPEN_b
     
		  while (1)
		  {
			  if(can_flag)
			  {
				  snd_can_status = 1;
		     	  can_send(45);
				  rcv_data();
				  can_flag=false;
			  }
		  }

     
}
//      while (1)
//      {
// 		 	PORTC.DIRCLR = PIN2_bm;
// 		 	PORTC.PIN2CTRL |= PORT_PULLUPEN_bm;
// 		 	if (PORTC.IN & PIN2_bm)
// 		 	{
// 		     snd_can_status = 1;
// 		     can_send(45);
// 		   
// 	        }
	      //   rcv_data();
// 		 }
// }

