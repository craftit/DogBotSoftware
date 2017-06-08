
#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>

// Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta
// as per the magnitude invariant clarke transform
// The magnitude of the alpha-beta vector may not be larger than sqrt(3)/2
// Returns 0 on success, and -1 if the input was out of range
int SVM(float alpha, float beta, float* tA, float* tB, float* tC);

// Magnitude must not be larger than sqrt(3)/2, or 0.866
void svm2(float alpha, float beta, uint32_t PWMHalfPeriod,
    uint32_t* tAout, uint32_t* tBout, uint32_t* tCout);

#endif //__UTILS_H
