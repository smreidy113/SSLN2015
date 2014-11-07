/*
 * BreazeControl.c
 *
 * Created: 11/6/2014 9:37:08 PM
 *  Author: Sean
 */ 

#define F_CPU 16000000UL
#include "m_general.h"

#include <avr/io.h>

int i = 0;
int flag = 0;
int ADCarr[8] = {0, 0, 0, 0, 0, 0, 0, 0};

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

int main(void)
{
    while(1)
    {
        m_wait(100);
		m_red(TOGGLE);
		getADC();
    }
}

ISR(ADC_vect) {
	//cli();
	conversion = 1;
	//sei();
}