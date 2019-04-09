#ifndef MATHFUNC_HEADER
#define MATHFUNC_HEADER 1

#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include <arm_math.h>

#ifndef M_PI
#define M_PI (3.14159265359)
#endif

static const float one_by_sqrt3 = 0.57735026919f;
//static const float two_by_sqrt3 = 1.15470053838f;
static const float sqrt3_by_2 = 0.86602540378;

inline float Rad2Deg(float deg)
{ return deg * 180/M_PI; }

inline int sqr(int val)
{
   return val * val;
}

inline float mysqrtf(float op1)
{
  if(op1 <= 0.f)
    return 0.f;

   float result;
   __ASM volatile ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
   return (result);
}

#endif
