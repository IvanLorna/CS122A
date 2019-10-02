/*
NAME:Ivan Lorna
Email: ilorn001@ucr.edu
Partner: Emanuel Halfon
*/ 

#include <avr/io.h>
#include <stdlib.h>
#include "../../../Headers/lcd.h"
#include "../../../Headers/scheduler.h"

#define A0 (~PINA & 0x01)

volatile unsigned char port_B = 0x00;
unsigned char on_off = 0x00;
unsigned char flag = 0x00;
unsigned  char score = 0x00;
const unsigned long PERIOD = 250;

unsigned char SetBit(unsigned char pin, unsigned char number, unsigned char bin_value)
{
	return (bin_value ? pin | (0x01 << number) : pin & ~(0x01 << number));
}

//TASK 1
enum ButtonStates {Start_On, On, Off};
int Button(int state)
{
	LCD_Cursor(1);
	LCD_WriteData('0' + score);
	switch(state)
	{
		case Start_On:
		state = Off;
		break;
		case On:
		state = A0 ? On : Off;
		break;
		case Off:
		state = A0 ? On : Off;
		break;
		default:
		state = Start_On;
		break;
	}
	switch(state)
	{
		case On:
		if(flag)
		{
			score++;
		}
		else
		{
			if(score > 0)
			{
				score--;
			}
		}
		break;
	}
	return state;
}

//TASK 2
enum LightOneStates {Start_one, Light1, Light2, Light3};
int LightStates(int state)
{
	switch(state)
	{
		case Start_one:
		state = on_off ? state : Light1;
		break;
		case Light1:
		state = on_off ? state : Light2;
		flag = 0x01;
		break;
		case Light2:
		state = on_off ? state : Light3;
		flag = 0x00;
		break;
		case Light3:
		state = on_off ? state : Light1;
		break;
		default:
		state = Start_one;
		break;
	}
	switch(state)
	{
		case Light1:
		port_B = SetBit(port_B,2, 0);
		port_B = SetBit(port_B, 0, 1);
		break;
		case Light2:
		port_B = SetBit(port_B,0,0);
		port_B = SetBit(port_B, 1, 1);
		break;
		case Light3:
		port_B = SetBit(port_B,1,0);
		port_B = SetBit(port_B, 2, 1);
		break;
		default:
		port_B = port_B;
	}
	return state;
}

//TASK 3
enum LightOn_OFFStates {Start, LightOn, LightOff};
int LightOn_OFF(int state)
{
	switch(state)
	{
		case Start:
		state = LightOn;
		break;
		case LightOn:
		state = on_off ? state : LightOff;
		break;
		case LightOff:
		state = on_off ? state : LightOn;
		break;
		default:
		state = Start;
	}
	switch(state)
	{
		case LightOn:
		port_B = SetBit(port_B,3,1);
		break;
		case LightOff:
		port_B = SetBit(port_B, 3, 0);
		break;
		default:
		port_B = port_B;
	}
	return state;
}

int main(void)
{
	//initialize ports
	DDRB = 0xFF; PORTB = 0x00;
	DDRA = 0x00; PORTA = 0xFF;
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	
	//initialize tasks
	task tasks[3];
	
	//lights task
	unsigned char i = 0;
	tasks[i].state = Start_one;
	tasks[i].period = 500;
	tasks[i].elapsedTime = 0;
	tasks[i].TickFct = &LightStates;
	i++;
	
	//button task
	tasks[i].state = Start;
	tasks[i].period = 1000;
	tasks[i].elapsedTime = 0;
	tasks[i].TickFct = &LightOn_OFF;
	i++;
	
	//LCD task
	tasks[i].state = Start_On;
	tasks[i].period = 250;
	tasks[i].elapsedTime = 0;
	tasks[i].TickFct = &Button;
	
	//initialize LCD
	LCD_init();
	LCD_ClearScreen();
	TimerSet(PERIOD);
	TimerOn();
	
	while(1)
	{
		PORTB = port_B;
	}
	return 0;
}


