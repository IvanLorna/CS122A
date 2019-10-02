/*
NAME:Ivan Lorna
Email: ilorn001@ucr.edu
Partner: Emanuel Halfon

Lab 1, Part 1
*/
#include <avr/io.h>
#include <stdlib.h>
#include "../../../Headers/scheduler.h"

#define A0 (~PINA & 0x01)

volatile unsigned char port_B = 0x00;
unsigned char on_off = 0x00;

const unsigned long PERIOD = 50;

unsigned char SetBit(unsigned char pin, unsigned char number, unsigned char bin_value)
{
	return (bin_value ? pin | (0x01 << number) : pin & ~(0x01 << number));
}

enum ToggleButtonStates {Start_On, PressOn, PressOff, On, Off};
int ToggleButton(int state)
{
	switch(state)
	{
		case Start_On:
		state = Off;
		break;
		case On:
		state = A0 ? PressOff : On;
		break;
		case Off:
		state = A0 ? PressOn : Off;
		break;
		case PressOn:
		state =  A0 ? state : On;
		break;
		case PressOff:
		state = A0 ? state : Off;
		break;
		default:
		state = Start_On;
		break;
	}
	switch(state)
	{
		case On:
		on_off = 0x01;
		break;
		case Off:
		on_off = 0x00;
		break;
	}
	return state;
}

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
		break;
		case Light2:
		state = on_off ? state : Light3;
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
	DDRB = 0xFF; PORTB = 0x00;
	DDRA = 0xF0; PORTA = 0x0F;
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	
	task tasks[3];
	
	unsigned char i = 0;
	tasks[i].state = Start_one;
	tasks[i].period = 500;
	tasks[i].elapsedTime = 0;
	tasks[i].TickFct = &LightStates;
	i++;
	
	tasks[i].state = Start;
	tasks[i].period = 1000;
	tasks[i].elapsedTime = 0;
	tasks[i].TickFct = &LightOn_OFF;
	i++;
	
	tasks[i].state = Start_On;
	tasks[i].period = 50;
	tasks[i].elapsedTime = 0;
	tasks[i].TickFct = &ToggleButton;
	
	TimerSet(PERIOD);
	TimerOn();
	
	while(1)
	{
		PORTB = port_B;
	}
