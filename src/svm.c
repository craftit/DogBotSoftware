
#include "svm.h"

static const float one_by_sqrt3 = 0.57735026919f;
static const float two_by_sqrt3 = 1.15470053838f;

int SVM(float alpha, float beta, float* tA, float* tB, float* tC) {
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;

            break;
        }
        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;

            break;
        }
        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;

            break;
        }
        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;

            break;
        }
        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;

            break;
        }
        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;

            break;
        }
    }

    int retval = 0;
    if (
           *tA < 0.0f
        || *tA > 1.0f
        || *tB < 0.0f
        || *tB > 1.0f
        || *tC < 0.0f
        || *tC > 1.0f
    ) retval = -1;
    return retval;
}

#define ONE_BY_SQRT3                    (0.57735026919)
#define TWO_BY_SQRT3                    (2.0f * 0.57735026919)
#define SQRT3_BY_2                      (0.86602540378)

// Magnitude must not be larger than sqrt(3)/2, or 0.866
void svm2(float alpha, float beta, uint32_t PWMHalfPeriod,
                uint32_t* tAout, uint32_t* tBout, uint32_t* tCout)
{
        uint32_t sector;

        if (beta >= 0.0f) {
                if (alpha >= 0.0f) {
                        //quadrant I
                        if (ONE_BY_SQRT3 * beta > alpha)
                                sector = 2;
                        else
                                sector = 1;
                } else {
                        //quadrant II
                        if (-ONE_BY_SQRT3 * beta > alpha)
                                sector = 3;
                        else
                                sector = 2;
                }
        } else {
                if (alpha >= 0.0f) {
                        //quadrant IV5
                        if (-ONE_BY_SQRT3 * beta > alpha)
                                sector = 5;
                        else
                                sector = 6;
                } else {
                        //quadrant III
                        if (ONE_BY_SQRT3 * beta > alpha)
                                sector = 4;
                        else
                                sector = 5;
                }
        }

        // PWM timings
        uint32_t tA, tB, tC;

        switch (sector) {

        // sector 1-2
        case 1: {
                // Vector on-times
                uint32_t t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
                uint32_t t2 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

                // PWM timings
                tA = (PWMHalfPeriod - t1 - t2) / 2;
                tB = tA + t1;
                tC = tB + t2;

                break;
        }

        // sector 2-3
        case 2: {
                // Vector on-times
                uint32_t t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
                uint32_t t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

                // PWM timings
                tB = (PWMHalfPeriod - t2 - t3) / 2;
                tA = tB + t3;
                tC = tA + t2;

                break;
        }

        // sector 3-4
        case 3: {
                // Vector on-times
                uint32_t t3 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
                uint32_t t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

                // PWM timings
                tB = (PWMHalfPeriod - t3 - t4) / 2;
                tC = tB + t3;
                tA = tC + t4;

                break;
        }

        // sector 4-5
        case 4: {
                // Vector on-times
                uint32_t t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
                uint32_t t5 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

                // PWM timings
                tC = (PWMHalfPeriod - t4 - t5) / 2;
                tB = tC + t5;
                tA = tB + t4;

                break;
        }

        // sector 5-6
        case 5: {
                // Vector on-times
                uint32_t t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
                uint32_t t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

                // PWM timings
                tC = (PWMHalfPeriod - t5 - t6) / 2;
                tA = tC + t5;
                tB = tA + t6;

                break;
        }

        // sector 6-1
        case 6: {
                // Vector on-times
                uint32_t t6 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
                uint32_t t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

                // PWM timings
                tA = (PWMHalfPeriod - t6 - t1) / 2;
                tC = tA + t1;
                tB = tC + t6;

                break;
        }
        }

        *tAout = tA;
        *tBout = tB;
        *tCout = tC;
}

