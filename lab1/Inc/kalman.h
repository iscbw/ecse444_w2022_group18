typedef struct kalman_state {
	float q;	// process noise covariance
	float r;	// measurement noise covariance
	float x;	// estimated value
	float p;	// estimation error covariance
	float k;	// adaptive Kalman filter gain
} kalman_state;

extern float kalman(kalman_state* kstate, float measurement);
