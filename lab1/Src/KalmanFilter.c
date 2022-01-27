#include "KalmanFilter.h"

int Kalmanfilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	for (int i=0; i<Length; i++) {
		*(OutputArray+i*4) = kalman(kstate, *(InputArray+i*4));
	}
}

