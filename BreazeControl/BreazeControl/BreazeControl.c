/*
 * BreazeControl.c
 *
 * Created: 11/6/2014 9:37:08 PM
 *  Author: Sean
 */ 

#define F_CPU 16000000UL
#include "m_general.h"

#include <avr/io.h>

int main(void)
{
    while(1)
    {
        m_wait(50);
		m_red(TOGGLE);
    }
}