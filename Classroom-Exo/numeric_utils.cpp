#include "numeric_utils.h"

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapFloat(int x, float in_min, float in_max, float out_min, float out_max) {
    return mapFloat((float)x, in_min, in_max, out_min, out_max);
}