/*
 * kalman.s
 */

.syntax unified
.section .text
.global kalman

/*
 * s4:	q
 * s5:	r
 * s6:	x
 * s7:	p
 * s8:	k
 */

kalman:	// (kalman_state* kstate, float measurement)
	vstmdb		sp!, {s3-s8}	// push
	vldmia		r0, {s4-s8}		// multiple load floating point number from the struct input
	vadd.f32	s7, s7, s4		// p=p+q
	vadd.f32	s3, s7, s5		// r3 as a intermediate value, r3=p+r
	vdiv.f32	s8, s7, s3		// k=p/(p+r)
	vsub.f32	s3, s0, s6		// r3 = measurement-x
	vmla.f32	s6, s8, s3		// x = x + k*(measurement-x))
	vmov.f32	s3, #1.0
	vsub.f32	s3, s3, s8		// r3=1-k
	vmul.f32	s7, s3, s7		// p=(1-k)*p

	// detect overflow & underflow
	push		{r4}
	vmrs		r4, FPSCR		// FPSCR: floating-point status and control register
	tst			r4, #0b01100	// test overflow & underflow
	it			ne
	andne		r4, r4, #0xfffffff0	// clear exception flags
	bne			return			// branch to handle exception

	vstmia		r0, {s4-s8}		// store all values back

return:	// if overflow, use the original state then return measurement
	vmov.f32	s0, s6			// return x
	pop			{r4}
	vldmia		sp!, {s3-s8}	// pop
	bx			lr

.end
