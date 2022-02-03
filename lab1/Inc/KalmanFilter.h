#include "kalman.h"

int Kalmanfilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);

int Kalmanfilter_c(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);

float kalman_c(kalman_state* state, float measurement);


