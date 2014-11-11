#include "control.h"
#include <stdlib.h>

int oppDir(int st) {
	if (st == 1) {
		return 2;
		} else {
		return 1;
	}
}

CtrlOutput *straight_control(float *ADCs) {
	int thresholdlow = 700;
	int thresholdhigh = 800;
	unsigned int maxADC;
	CtrlOutput *ans = malloc(sizeof(CtrlOutput));
	if (ADCs[0] > ADCs[1]) {
		ans->state = FORWARD;
		ans->orientation = 0;
		maxADC = ADCs[0];
	}
	else {
		ans->state = BACKWARD;
		ans->orientation = 0;
		maxADC = ADCs[1];
	}
	
	if (maxADC < thresholdhigh && maxADC > thresholdlow) {
		ans->state = WAIT;
		ans->orientation = 0;
		ans->speed = 0;
	}
	else if (maxADC >= thresholdhigh) {
		ans->state = oppDir(ans->state);
		ans->speed = 1.0 - (1023 - maxADC) / (1023 - thresholdhigh);
	}
	else {
		ans->speed = 1.0 - (maxADC / thresholdlow);
	}
	
	return ans;
}