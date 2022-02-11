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

 /*
  * Kalman filter function that uses cmsis-dsp
  */
int Kalmanfilter_cmsis(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	for (int i=0; i<Length; i++) {
		OutputArray[i] = kalman_cmsis(kstate, InputArray[i]);
	}
	return 0;
}

/*
 * Kalman filter rewritten in C and cmsis-dsp functions
 */
float kalman_cmsis(kalman_state* state, float measurement) {
	float a = 0.0;	//p + r
	float b = 0.0;	//measurement - x
	float c = 0.0;	//k * (measurement - x)
	float d = 0.0;	//(1-k)
	float constant = 1;

	arm_add_f32(&state->p, &state->q, &state->p, 1);
	arm_add_f32(&state->p, &state->r, &a, 1);
	state->k = (state->p) / a;
	arm_sub_f32(&measurement, &state->x, &b, 1);
	arm_mult_f32(&state->k, &b, &c, 1);
	arm_add_f32(&state->x, &c, &state->x, 1);
	arm_sub_f32(&constant, &state->k, &d, 1);
	arm_mult_f32(&d, &state->q, &state->p, 1);
}


