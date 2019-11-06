/*
 * MinisubMatrix.c
 *
 * Created: 11/6/2019 9:24:03 AM
 * Author : ivanl
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include "scheduler.h"

//GLOBAL
unsigned char LED_ARRAY[8];


//SMs

//Reads PWM signal being sent into PINA and updates the array of characters
int ReadPWM(int state) { //state acts as counter
	unsigned char temp;
	
	//read current REG_A contents
	temp = PINA;
	
	//write data into array of characters
	LED_ARRAY[state] = temp;
	
	//update state as a counter
	state = (state >= 7) ? 0 : state+1; //state should only be 0-7 going back to 0 after 7
	
	//return state
	return state;		
}

//reads from array of characters to send data to LED matrix
int UpdateMatrix(int state) {//this state should always be same value as READPWM's state. this is just to use it as an iterator/counter.
	
	//write to PORTC the current iterated character data
	PORTC = LED_ARRAY[state];
	
	//write to portD such that only one row of the matrix is 'grounded' at a time, activating LEDS
	PORTD = ~(0x01 << state);
	
	
	//return state updating it the same way ReadPWM does
	state = (state >= 7) ? 0 : state+1; //state should only be 0-7 going back to 0 after 7
	return state;
}

//FIXME::have emanual show you how to set up a button ISR for fixing desync
//or find a software solution to desync

int main(void)
{
   DDRA = 0x00; PORTA = 0xFF; //PINA as input
   
   DDRC = 0xFF; PORTC = 0x00; //PORTC as output
   DDRD = 0xFF; PORTD = 0x00; //PORTD as output
   
   //init task 0 as ReadPWM
   tasks[0].state = 0;
   tasks[0].period = 250;
   tasks[0].elapsedTime = 0;
   tasks[0].TickFct = &ReadPWM;
   
   //init task 1 as UpdateMatrix
   tasks[1].state = 0;
   tasks[1].period = 250;
   tasks[1].elapsedTime = 0;
   tasks[1].TickFct = &UpdateMatrix;
   
    while (1) 
    {
		//kick jeff's dog
    }
}


