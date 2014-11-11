#include <stdlib.h>

#define WAIT 0
#define FORWARD 1
#define BACKWARD 2

typedef struct CtrlOutput_tag {
	int state;
	float orientation;
	float speed;
	} CtrlOutput;
	
CtrlOutput *straight_control(float *ADCs);