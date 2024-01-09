/*
 * Hyleg.h
 *
 */

#ifndef ICODE_HYLEG_HYLEG_H_
#define ICODE_HYLEG_HYLEG_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

float* FK(float theta1,float theta2);
void calVelocity(float q1, float q2, float dq1, float dq2, float* velocity);
#endif /* ICODE_HYLEG_HYLEG_H_ */
