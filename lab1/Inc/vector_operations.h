void vec_sub(float* op1, float* op2, float* result, int length);

float vec_mean(float* input, int length);

float vec_sd(float* input, int length);

float vec_correlation(float* in1, float* in2, int length);

void vec_convolution(float* in1, float* in2, float* result, int length);
