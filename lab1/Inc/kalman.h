#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

typedef struct kalman_state {
	float q;	// process noise covariance
	float r;	// measurement noise covariance
	float x;	// estimated value
	float p;	// estimation error covariance
	float k;	// adaptive Kalman filter gain
} kalman_state;

/*
 * Kalman filter implemented in assembly
 */
extern float kalman(kalman_state* kstate, float measurement);

#endif
