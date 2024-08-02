#include <stdint.h>
#include "math_ops.h"

// useful conversions
#define rad2pulse(x) float(x) * (4096.0f/(2*PI)) + 2048 // 0 rad = 2048 = upright pos // Motor -> CW - , CCW + // -pi < x < pi
#define pulse2rad(x) (float(x)-2048) * ((2*PI)/4096.0f) // check that this works
#define rpm2rads(x) float(x)*(0.229f*2.0f*PI)/60.0f // = 0.0239
