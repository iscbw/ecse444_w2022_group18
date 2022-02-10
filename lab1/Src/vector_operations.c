#include <math.h>
#include "vector_operations.h"

/*
 * calculate the difference between two vectors without using cmsis
 */
void vec_sub(float* op1, float* op2, float* result, int length) {
	for (int i=0; i<length; i++) {
		result[i] = op1[i] - op2[i];
	}
}

/*
 * calculate the mean of a vector input without using cmsis
 */
float vec_mean(float* input, int length) {
	float sum = 0;

	for (int i=0; i<length; i++) {
		sum += input[i];
	}

	return sum/length;
}

/*
 * calculate the standard deviation of a vector without using cmsis
 */
float vec_sd(float* input, int length) {
	float mean = vec_mean(input, length);
	float var = 0;

	for (int i=0; i<length; i++) {
		var += powf(input[i]-mean, 2);
	}

	return sqrt(var/length);
}

/*
 * calculate the correlation between two vectors without using cmsis
 */
void vec_correlation(float* in1, float* in2, float* result, int length) {
	for (int n = 0; n < 2*length - 1; n++) {
		result[n] = 0;
		for (int k = 0; k < length; k++) {
			result[n] += (k < length ? in1[k] : 0) * ((length-1-n+k) < length ? in2[(length-1-n+k)] : 0);
		}
	}
}

/*
 * calculate the convolution between two vectore without using cmsis
 */
void vec_convolution(float* in1, float* in2, float* result, int length) {
	for (int n = 0; n < 2*length - 1; n++) {
		result[n] = 0;
		for (int k = 0; k < length; k++) {
			result[n] += (k < length ? in1[k] : 0) * (n - k < length ? in2[n - k] : 0);
		}
	}
}
