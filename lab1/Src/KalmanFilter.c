#include "KalmanFilter.h"
#include "arm_math.h"
#include <math.h>

/*
 * Kalman filter function that uses the assembly subroutine
 */
int Kalmanfilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	for (int i=0; i<Length; i++) {
		OutputArray[i] = kalman(kstate, InputArray[i]);
	}
	return 0;
}

/*
 * Kalman filter function that is in C
 */
int Kalmanfilter_c(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	for (int i=0; i<Length; i++) {
		OutputArray[i] = kalman_c(kstate, InputArray[i]);
	}
	return 0;
}

/*
 * Kalman filter rewritten in C
 */
float kalman_c(kalman_state* state, float measurement) {
	state->p += state->q;
	state->k = state->p / (state->p + state->r);
	state->x = state->x + state->k * (measurement - state->x);
	state->p = (1 - state->k) * state->p;

	return state->x;
}


