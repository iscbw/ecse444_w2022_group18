#include <math.h>
#include "vector_operations.h"

/*
 * calculate the difference between two vectors without using cmsis
 */
void vec_sub(float* op1, float* op2, float* result, int length) {
	for (int i=0; i<Length; i++) {
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
float vec_correlation(float* in1, float* in2, int length) {
	float mean1 = vec_mean(in1, length);
	float mean2 = vec_mean(in2, length);
	float covar = 0;

	for (int i=0; i<length; i++) {
		covar += (in1[i]-mean1) * (in2[i]-mean2);
	}

	covar /= length;

	float sd1 = vec_sd(in1, length);
	float sd2 = vec_sd(in2, length);

	return covar/(sd1*sd2);
}

/*
 * calculate the convolution between two vectore without using cmsis
 */
void vec_convolution(float* in1, float* in2, float* result) {
	for (int n = 0; n < 2*length - 1; n++) {
		int kmin, kmax, k;

		result[n] = 0;

		kmin = (n >= length - 1) ? n - (length - 1) : 0;
		kmax = (n < length - 1) ? n : length - 1;

		for (k = kmin; k <= kmax; k++) {
			result[n] += in1[k] * in2[n - k];
		}
	}
}
