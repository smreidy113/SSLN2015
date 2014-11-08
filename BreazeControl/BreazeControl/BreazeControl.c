/*
 * BreazeControl.c
 *
 * Created: 11/6/2014 9:37:08 PM
 *  Author: Sean, Patrik
 */ 


#define WAIT 0
#define FORWARD 1
#define BACKWARD 2
#define TURNLEFT 3
#define TURNRIGHT 4
#define ROTATELEFT 5
#define ROTATERIGHT 6

#define F_CPU 16000000UL
#include "m_general.h"

#include <avr/io.h>

int i = 0;
int flag = 0;
int ADCarr[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
char state = 0;
float spd = 0;

void chooseInput(int i) {
	switch (i) {
		case 0:
		clear(ADCSRB, MUX5);//Set analog input (F0)
		clear(ADMUX, MUX2);	//^
		clear(ADMUX, MUX1);	//^
		clear(ADMUX, MUX0);	//^

		break;
		case 1:
		clear(ADCSRB, MUX5);//Set analog input (F1)
		clear(ADMUX, MUX2);	//^
		clear(ADMUX, MUX1);	//^
		set(ADMUX, MUX0);	//^

		break;
		case 2:
		clear(ADCSRB, MUX5);//Set analog input (F4)
		set(ADMUX, MUX2);	//^
		clear(ADMUX, MUX1);	//^
		clear(ADMUX, MUX0);	//^

		break;
		case 3:

		clear(ADCSRB, MUX5);//Set analog input (F5)
		set(ADMUX, MUX2);	//^
		clear(ADMUX, MUX1);	//^
		set(ADMUX, MUX0);	//^
		break;
		case 4:
		clear(ADCSRB, MUX5);//Set analog input (F6)
		set(ADMUX, MUX2);	//^
		set(ADMUX, MUX1);	//^
		clear(ADMUX, MUX0);	//^
		break;
		case 5:
		set(ADCSRB, MUX5);//Set analog input (D4)
		clear(ADMUX, MUX2);	//^
		clear(ADMUX, MUX1);	//^
		clear(ADMUX, MUX0);	//^
		break;
		case 6:
		set(ADCSRB, MUX5);//Set analog input (D6)
		clear(ADMUX, MUX2);	//^
		clear(ADMUX, MUX1);	//^
		set(ADMUX, MUX0);	//^
		break;
		case 7:
		clear(ADCSRB, MUX5);//Set analog input (F7)
		set(ADMUX, MUX2);
		set(ADMUX, MUX1);
		set(ADMUX, MUX0);
		i=-1;
		break;
	}
}

int ADC0 = 0;
int ADC1 = 0;
int ADC2 = 0;
int ADC3 = 0;
int ADC4 = 0;
int ADC5 = 0;
int ADC6 = 0;
int ADC7 = 0;
int conversion = 0;

void getADC() {
	if (conversion) {
		switch (flag) {
			case 0:
			ADC0 = ADC;
			break;
			case 1:
			ADC1 = ADC;
			break;
			case 2:
			ADC2 = ADC;
			break;
			case 3:
			ADC3 = ADC;
			break;
			case 4:
			ADC4 = ADC;
			break;
			case 5:
			ADC5 = ADC;
			break;
			case 6:
			ADC6 = ADC;
			break;
			case 7:
			ADC7 = ADC;
			break;
		}
		clear(ADCSRA, ADEN);	//Enable/Start conversion
		clear(ADCSRA, ADSC);	//^
		clear(ADCSRA, ADATE);
		clear(ADCSRA, ADIF);
		if (flag >= 0 && flag < 8) {
			flag = (flag + 1) % 8;
			chooseInput(flag);
		}
		set(ADCSRA, ADATE);	//Set trigger to free-running mode
		set(ADCSRA, ADEN);	//Enable/Start conversion
		set(ADCSRA, ADSC);	//^
		
		set(ADCSRA, ADIF);	//Enable reading results
		conversion = 0;
	}

	ADCarr[0] = ADC0;
	ADCarr[1] = ADC1;
	ADCarr[2] = ADC2;
	ADCarr[3] = ADC3;
	ADCarr[4] = ADC4;
	ADCarr[5] = ADC5;
	ADCarr[6] = ADC6;
	ADCarr[7] = ADC7;
	
	
}

void drive_straight(char dir, float speed) {
	switch (dir) {
		case FORWARD:
			set(PORTB, 2);
			set(PORTB, 3);
			break;
		case BACKWARD:
			clear(PORTB, 2);
			clear(PORTB, 3);
			break;
	}
	OCR1B = (unsigned int) (OCR1A * speed);
	OCR3B = (unsigned int) (ICR3 * speed);
}

int oppDir(int st) {
	if (st == 1) {
		return 2;
	} else {
		return 1;
	}
}

int main(void)
{
	
	//TIMER 1: for left wheel
	set(TCCR1B, WGM13);
	set(TCCR1B, WGM12);
	set(TCCR1A, WGM11);
	set(TCCR1A, WGM10);
	
	set(TCCR1A, COM1B1);
	clear(TCCR1A, COM1B0);
	
	clear(TCCR1B, CS12);
	clear(TCCR1B, CS11);
	set(TCCR1B, CS10);
	
	OCR1A = 0xFFFF;
	OCR1B = 0;
	
	
	//TIMER 3: For right wheel
	set(TCCR3B, WGM33);
	set(TCCR3B, WGM32);
	set(TCCR3A, WGM31);
	clear(TCCR3A, WGM30);
	
	set(TCCR3A, COM3A1);
	clear(TCCR3A, COM3A0);
	
	clear(TCCR3B, CS32);
	clear(TCCR3B, CS31);
	set(TCCR3B, CS30);
	
	ICR3 = 0xFFFF;
	OCR3A = 0;
	
	set(DDRB,6);
	set(DDRC,6);
	
	set(DDRB,2);
	set(DDRB,3);
	
	int thresholdlow = 700;
	int thresholdhigh = 800;
	
	int maxADC = 0;
	
    while(1)
    {
        m_wait(100);
		m_red(TOGGLE);
		getADC();
		
		if (ADCarr[0] > ADCarr[1]) {
			state = FORWARD;
			maxADC = ADCarr[0];
		}
		else {
			state = BACKWARD;
			maxADC = ADCarr[1];
		}
		
		if (maxADC < thresholdhigh && maxADC > thresholdlow) {
			state = WAIT;
		}
		else if (maxADC >= thresholdhigh) {
			state = oppDir(state);
			spd = 1.0 - (1023 - maxADC) / (1023 - thresholdhigh);
		}
		else {
			spd = 1.0 - (maxADC / thresholdlow);
		}
		
		switch (state) {
			case FORWARD:
				drive_straight(FORWARD, spd);
				break;
			case BACKWARD:
				drive_straight(BACKWARD, spd);
				break;
		}
		
    }
}

ISR(ADC_vect) {
	//cli();
	conversion = 1;
	//sei();
}