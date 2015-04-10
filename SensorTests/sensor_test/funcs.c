#include "funcs.h"

/* Measures the length (in microseconds) of a pulse on the pin; state is HIGH
 * or LOW, the type of pulse to measure.  Works on pulses from 2-3 microseconds
 * to 3 minutes in length, but must be called at least a few dozen microseconds
 * before the start of the pulse. */
void pulseInMult(unsigned long *pulses, uint8_t pin1, uint8_t pin2, uin8_t pin3, uint8_t state, unsigned long timeout)
{
	
	// cache the port and bit of the pin in order to speed up the
	// pulse width measuring loop and achieve finer resolution.  calling
	// digitalRead() instead yields much coarser resolution.
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	uint8_t stateMask = (state ? bit : 0);
	unsigned long width = 0; // keep initialization out of time critical area
	
	// convert the timeout from microseconds to a number of times through
	// the initial loop; it takes 16 clock cycles per iteration.
	unsigned long numloops1 = 0;

	unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;
	
	// wait for any previous pulse to end
	while ((*portInputRegister(port) & bit) == stateMask)
		if (numloops++ == maxloops)
			return 0;
	
	// wait for the pulse to start
	while ((*portInputRegister(port) & bit) != stateMask)
		if (numloops++ == maxloops)
			return 0;
	
	// wait for the pulse to stop
	while ((*portInputRegister(port) & bit) == stateMask) {
		if (numloops++ == maxloops)
			return 0;
		width++;
	}

	// convert the reading to microseconds. There will be some error introduced by
	// the interrupt handlers.

	// Conversion constants are compiler-dependent, different compiler versions
	// have different levels of optimization.
#if __GNUC__==4 && __GNUC_MINOR__==3 && __GNUC_PATCHLEVEL__==2
	// avr-gcc 4.3.2
	return clockCyclesToMicroseconds(width * 21 + 16);
#elif __GNUC__==4 && __GNUC_MINOR__==8 && __GNUC_PATCHLEVEL__==1
	// avr-gcc 4.8.1
	return clockCyclesToMicroseconds(width * 24 + 16);
#elif __GNUC__<=4 && __GNUC_MINOR__<=3
	// avr-gcc <=4.3.x
	#warning "pulseIn() results may not be accurate"
	return clockCyclesToMicroseconds(width * 21 + 16);
#else
	// avr-gcc >4.3.x
	#warning "pulseIn() results may not be accurate"
	return clockCyclesToMicroseconds(width * 24 + 16);
#endif

}
