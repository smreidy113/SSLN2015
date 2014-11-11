#include "point.h"
#include <math.h>

float dist(Point *a) {
	return sqrt(pow(a->x, 2) + pow(a->y, 2));
}