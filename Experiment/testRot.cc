
#include "Ravl/ParseCSV.hh"
#include "Ravl/SysLog.hh"
#include "Ravl/Random.hh"
#include "Ravl/Vector3d.hh"
#include "Ravl/StrStream.hh"
#include <cstdint>
#include <math.h>


float mysqrtf(float val) {
  return sqrt(val);
}
#define USE_STANDARD 0
#if USE_STANDARD
// In use
uint16_t g_phaseAngles[12][3] = {
{2123,2290,2409},
{1997,2183,2409},
{1980,2079,2373},
{1972,1970,2223},
{1978,1890,2069},
{2102,1936,2013},
{2229,2008,1993},
{2367,2139,2001},
{2381,2233,2039},
{2391,2340,2206},
{2384,2412,2357},
{2265,2365,2397}
};
#else

uint16_t g_phaseAngles[12][3] = {
{2626,2902,2995},
{2402,2767,3024},
{2331,2590,3006},
{2290,2375,2795},
{2263,2224,2549},
{2418,2259,2426},
{2628,2328,2364},
{2861,2447,2331},
{2942,2601,2331},
{2989,2835,2532},
{3008,2997,2766},
{2851,2977,2915}
};

#endif

float hallToAngle(uint16_t *sensors);
float hallToAngleNorm(uint16_t *sensors);
float hallToAngleDot(uint16_t *sensors);
float hallToAngleDot2(uint16_t *sensors);
float hallToAngleDot3(uint16_t *sensors);
float hallToAngleDot4(uint16_t *sensors);
float hallToAngleTan(uint16_t *sensors);
float hallToAngleXTan(uint16_t *sensors);

float sqr(float val)
{ return val * val; }

int hallMean[3];
float g_phaseDistance[12];

float g_phaseAnglesNorm[12][3];
float g_phaseAnglesNormOrg[12][3];
float g_phaseDistanceNorm[12];

float originOffset = -2000;


const float g_projDir1[3] = { -0.408248,0.816497,-0.408248};
//const float g_projDir2[3] = { -0.377393 -0.845665 -0.377393 };
const float g_projDir2[3] = { -1.22474,-0,1.22474 };

float g_phaseAnglesProj[12][2];
float g_projMean[2];
float g_phaseDistanceProj[12];

void init()
{
  {
    RavlN::Vector3dC vec1(1.0,1.0,1.0);
    RavlN::Vector3dC vec2(g_projDir1[0],g_projDir1[1],g_projDir1[2]);
    vec2.MakeUnit();
    RavlN::Vector3dC cp = vec1.Cross(vec2);
    RavlDebug("Vec1:%s",RavlN::StringOf(vec2).c_str());
    RavlDebug("CrossP:%s",RavlN::StringOf(cp).c_str());
  }

  int lastIndex = 11;
  float sumx = 0;
  float normx = 0;
  for(int k = 0;k < 3;k++) {
    hallMean[k] = 0;
    sumx += g_projDir1[k] * g_projDir2[k];
    normx += sqr(g_projDir1[k]);
  }
  RavlDebug("Sumx:%f Mag:%f ",sumx,normx);

  g_projMean[0] = 0;
  g_projMean[1] = 0;
  for(int i = 0;i < 12;i++) {
    int sum = 0;
    float sumMag = 0;
    g_phaseAnglesProj[i][0] = 0;
    g_phaseAnglesProj[i][1] = 0;

    for(int k = 0;k < 3;k++) {
      hallMean[k] += g_phaseAngles[i][k];
      sum += sqr(g_phaseAngles[i][k] - g_phaseAngles[lastIndex][k]);
      sumMag += sqr(g_phaseAngles[i][k] - originOffset);

      g_phaseAnglesProj[i][0] += g_projDir1[k] * g_phaseAngles[i][k];
      g_phaseAnglesProj[i][1] += g_projDir2[k] * g_phaseAngles[i][k];
    }
#if 0
    float magP = mysqrtf(sqr(g_phaseAnglesProj[i][0]) + sqr(g_phaseAnglesProj[i][1]));
    g_phaseAnglesProj[i][0] /= magP;
    g_phaseAnglesProj[i][1] /= magP;
#endif

    g_projMean[0] += g_phaseAnglesProj[i][0];
    g_projMean[1] += g_phaseAnglesProj[i][1];

    sumMag = mysqrtf(sumMag);
    for(int k = 0;k < 3;k++) {
      g_phaseAnglesNormOrg[i][k] = (g_phaseAngles[i][k]-originOffset) / sumMag;
    }

    g_phaseDistance[i] = mysqrtf((float) sum);
    RavlDebug("Dist %f ",g_phaseDistance[i]);

    lastIndex = i;
  }
  for(int k = 0;k < 3;k++) {
    hallMean[k] /= 12.0;
  }
#if 0
  g_projMean[0] /= 12.0;
  g_projMean[1] /= 12.0;
#else
  g_projMean[0] = 0;
  g_projMean[1] = 0;
#endif

  lastIndex = 11;
  for(int i = 0;i < 12;i++) {
    float norm[3];
    float sum2 = 0;

    for(int k = 0;k < 3;k++) {
      norm[k] = g_phaseAngles[i][k] - hallMean[k];
      sum2 += sqr(norm[k]);
    }
    float mag = mysqrtf(sum2);

    for(int j = 0;j < 3;j++)
      g_phaseAnglesNorm[i][j] = norm[j] / mag;

    {
      float norm[2];
      norm[0] = g_phaseAnglesProj[i][0] - g_projMean[0];
      norm[1] = g_phaseAnglesProj[i][1] - g_projMean[1];
      float mag = sqrt(sqr(norm[0]) + sqr(norm[1]));
      //float mag = 1.0;
      g_phaseAnglesProj[i][0] = norm[0] / mag;
      g_phaseAnglesProj[i][1] = norm[1] / mag;
    }
  }

  for(int i = 0;i < 12;i++) {

    float sum = 0;
    for(int k = 0;k < 3;k++) {
      float diff = g_phaseAnglesNorm[i][k] - g_phaseAnglesNorm[lastIndex][k];
      sum += diff * diff;
    }
    g_phaseDistanceNorm[i] = mysqrtf((float) sum);
    g_phaseDistanceProj[i] = mysqrtf(sqr(g_phaseAnglesProj[i][0] - g_phaseAnglesProj[lastIndex][0]) +
                                      sqr(g_phaseAnglesProj[i][1] - g_phaseAnglesProj[lastIndex][1]));
    lastIndex = i;
  }


}


int main(int nargs,char **argv)
{
  init();


  RavlN::ParseCSVC csvFile;

  if(!csvFile.Open("cal1.csv")) {
    return false;
  }

  RavlN::SArray1dC<RavlN::StringC> values;
  const int numMethods = 8;

  float avErr[numMethods];
  float avAbsErr[numMethods];
  float avMagErr[numMethods];
  for(int k = 0;k < numMethods;k++) {
    avErr[k] = 0;
    avAbsErr[k] = 0;
    avMagErr[k] = 0;
  }

  uint16_t lastSensors[3];
  int lastAngle = 0;
  bool isFirst = true;
  int pnts = 0;
  while(true) {
    if(!csvFile.ReadValues(values))
      break;
    uint16_t sensorsIn[3];
    sensorsIn[0] = values[1].IntValue();
    sensorsIn[1] = values[2].IntValue();
    sensorsIn[2] = values[3].IntValue();
    int rval = values[0].IntValue();

    for(int i = 0;i < 2;i++)
    {
      float target;

      uint16_t sensors[3];
      if(i == 1) {
        for(int j = 0;j < 3;j++)
          sensors[j] = sensorsIn[j];
        target = (rval%12);
      } else {
        if(isFirst)
          break;
        float at = RavlN::Random1();
        for(int j = 0;j < 3;j++)
          sensors[j] = (sensorsIn[j] * at + lastSensors[j] * (1.0 -at));

        target = (rval * at + lastAngle * (1.0-at));
        target -= floor(target/12.0) * 12.0;
      }
      target *= 2.0;


      float estVal[numMethods];
      float errVal[numMethods];

#if 1
      estVal[0] = hallToAngle(sensors) + 1.04;
      estVal[1] = hallToAngleNorm(sensors) + 1.085;
      estVal[2] = hallToAngleDot(sensors) +1.032;
#if USE_STANDARD
      estVal[3] = hallToAngleDot2(sensors) + 1.09;// + 1.162;// + 0.017;
      estVal[4] = hallToAngleDot3(sensors) + 1.034;
      estVal[5] = hallToAngleDot4(sensors) + 1.22;
      estVal[6] = hallToAngleTan(sensors);
      estVal[7] = hallToAngleXTan(sensors);
#else
      estVal[3] = hallToAngleDot2(sensors);// + 1.162;// + 0.017;
      estVal[4] = hallToAngleDot3(sensors);
      estVal[5] = hallToAngleDot4(sensors);
      estVal[6] = hallToAngleTan(sensors);
      estVal[7] = hallToAngleXTan(sensors);
#endif
#else
      estVal[0] = hallToAngle(sensors);
      estVal[1] = hallToAngleNorm(sensors);
      estVal[2] = hallToAngleDot(sensors);
      estVal[3] = hallToAngleDot2(sensors);
      estVal[4] = hallToAngleDot3(sensors);
      estVal[5] = hallToAngleDot4(sensors);
      estVal[6] = hallToAngleTan(sensors);
      estVal[7] = hallToAngleXTan(sensors);
#endif

      for(int j = 0;j < numMethods;j++) {
        if(estVal[j] > 24.0) estVal[j] -= 24.0;

        float err = target - estVal[j];
        if(err > 12) err -= 24.0;
        if(err < -12) err += 24.0;
        avErr[j] += err;
        avAbsErr[j] += RavlN::Abs(err);
        avMagErr[j] += sqr(err);
        errVal[j] = err;
      }
      pnts++;
      RavlDebug("%d %f  %d %d %d  -> %f (%f)  %f (%f)  %f (%f)  %f (%f)",
                rval,
                target,
                (int) sensors[0],(int) sensors[1],(int) sensors[2],
                estVal[0],errVal[0],
                estVal[6],errVal[6]);
    }
    for(int j = 0;j < 3;j++)
      lastSensors[j] = sensorsIn[j];
    lastAngle = rval;
    isFirst = false;
  }


  for(int k = 0;k < numMethods;k++) {
    RavlDebug("%d Average error:%f  Abs:%f Mag:%f ",
              k,
            avErr[k]/(float)pnts,
            avAbsErr[k]/(float)pnts,
            sqrt(avMagErr[k]/(float)pnts)
            );
  }
}


void CoordinateNorm(const uint16_t *sensors,float *point)
{
#if 1
  const float v1[3] = { 3.1663, -12.8052, 15.2546 };
  const float v2[3] = { -21.9849, 15.5782, -13.649 };
  const float mean[2] = { -3.44823,11.7673 };

  point[0] = 0;
  point[1] = 0;
  float mag = 0;
  for(int i = 0;i < 3;i++) {
    mag += sqr(sensors[i]);
  }
  mag = sqrt(mag);
#else
  const float v1[3] = { -0.0011174,-0.00214568,0.00193329 };
  const float v2[3] = { -0.00121134, 0.00265257,0.00095082 };
  const float mean[2] = { 2.90756,-5.39188 };

  float mag = 1.0;
#endif
  for(int i = 0;i < 3;i++) {
    point[0] += v1[i] * ((float) sensors[i] / mag);
    point[1] += v2[i] * ((float) sensors[i] / mag);
  }
  point[0] += mean[0];
  point[1] += mean[1];

  mag = sqrt(sqr(point[0]) + sqr(point[1]));
  point[0] /= mag;
  point[1] /= mag;
}

float hallToAngleXTan(uint16_t *sensors)
{
  float norm[2];
  CoordinateNorm(sensors,norm);

  float distTable[12];
  int phase = 0;

  //RavlDebug("Vec: %f %f ",norm[0],norm[1]);

  float probe[2];
  CoordinateNorm(g_phaseAngles[0],probe);

  float minDist = probe[0] * norm[0] + probe[0] * norm[1];

  distTable[0] = minDist;

  for(int i = 1;i < 12;i++) {
    CoordinateNorm(g_phaseAngles[i],probe);

    float dist = (probe[0] * norm[0])  + (probe[1] * norm[1]);
    distTable[i] = dist;
    //RavlDebug("Dist:%f ",dist);
    if(dist > minDist) {
      phase = i;
      minDist = dist;
    }
  }
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(next > 11) next = 0;
  float lastDist2 = distTable[last];
  float nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = minDist - lastDist2;
  float nextDist = minDist - nextDist2;
  //RavlDebug("Last:%f  Next:%f ",lastDist,nextDist);
  angle += (nextDist-lastDist)/(nextDist + lastDist);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle;

}

float myatan2f( float y, float x )
{
    static const uint32_t sign_mask = 0x80000000;
    static const float b = 0.596227f;

    // Extract the sign bits
    uint32_t ux_s  = sign_mask & (uint32_t &)x;
    uint32_t uy_s  = sign_mask & (uint32_t &)y;

    // Determine the quadrant offset
    float q = (float)( ( ~ux_s & uy_s ) >> 29 | ux_s >> 30 );

    // Calculate the arctangent in the first quadrant
    float bxy_a = ::fabs( b * x * y );
    float num = bxy_a + y * y;
    float atan_1q =  num / ( x * x + bxy_a + num );

    // Translate it to the proper quadrant
    uint32_t uatan_2q = (ux_s ^ uy_s) | (uint32_t &)atan_1q;
    return q + (float &)uatan_2q;
}

float hallToAngleTan(uint16_t *sensors)
{
  float at[2];
  CoordinateNorm(sensors,at);

  RavlDebug("Vec:%f %f ",at[0],at[1]);
  //return myatan2f(at[1],at[0]) * 24.0 / (2.0 * M_PI);
  return atan2(at[0],at[1]) * 24.0 / (2.0 * M_PI);
}


float hallToAngleDot4(uint16_t *sensors)
{
  float distTable[12];
  int phase = 0;
  float norm[2];

  norm[0] = 0;
  norm[1] = 0;

  for(int i = 0;i < 3;i++) {
    norm[0] += ((float) sensors[i] - g_projMean[0] ) * g_projDir1[i];
    norm[1] += ((float) sensors[i] - g_projMean[0] ) * g_projDir2[i];
  }

  float mag = sqrt(sqr(norm[0]) + sqr(norm[1]));
  norm[0] /= mag;
  norm[1] /= mag;

  //RavlDebug("Vec: %f %f ",norm[0],norm[1]);

  mag = sqrt(sqr(g_phaseAnglesProj[0][0]) + sqr(g_phaseAnglesProj[0][1]));
  float maxCorr = ((g_phaseAnglesProj[0][0]/mag) * norm[0]) +
                  ((g_phaseAnglesProj[0][1]/mag) * norm[1]) ;

  distTable[0] = maxCorr;

  for(int i = 1;i < 12;i++) {
    //mag = mysqrtf(sqr(g_phaseAngles[i][0]) + sqr(g_phaseAngles[i][1]) + sqr(g_phaseAngles[i][2]));
    float mag = sqrt(sqr(g_phaseAnglesProj[i][0]) + sqr(g_phaseAnglesProj[i][1]));
    float corr = ((g_phaseAnglesProj[i][0]/mag) * norm[0]) +
                  ((g_phaseAnglesProj[i][1]/mag) * norm[1]);
    distTable[i] = corr;
    //RavlDebug("Corr:%f ",corr);
    if(corr > maxCorr) {
      phase = i;
      maxCorr = corr;
    }
  }
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(next > 11) next = 0;
  float lastDist2 = distTable[last];
  float nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = maxCorr - lastDist2;
  float nextDist = maxCorr - nextDist2;
  //Average error:0.007239  Abs:0.207922 Mag:0.263140
  float corr = (nextDist-lastDist)/(nextDist + lastDist);
  angle -= corr;
  //RavlDebug("Last:%f  Max:%f Next:%f Corr:%f ",lastDist,maxCorr,nextDist,corr);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle;

}


float hallToAngleDot3(uint16_t *sensors)
{
  float distTable[12];
  int phase = 0;
  float norm[2];

  norm[0] = 0;
  norm[1] = 0;

  for(int i = 0;i < 3;i++) {
    norm[0] += (float) sensors[i] * g_projDir1[i];
    norm[1] += (float) sensors[i] * g_projDir2[i];
  }

  norm[0] -= g_projMean[0];
  norm[1] -= g_projMean[1];

  //RavlDebug("Vec: %f %f ",norm[0],norm[1]);

  float minDist = sqr(g_phaseAnglesProj[0][0] - norm[0]) +
                sqr(g_phaseAnglesProj[0][1] - norm[1])
                ;

  distTable[0] = minDist;

  for(int i = 1;i < 12;i++) {
    float dist = sqr(g_phaseAnglesProj[i][0] - norm[0]) +
                  sqr(g_phaseAnglesProj[i][1] - norm[1]);
    distTable[i] = dist;
    //RavlDebug("Dist:%f ",dist);
    if(dist < minDist) {
      phase = i;
      minDist = dist;
    }
  }
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(next > 11) next = 0;
  float lastDist2 = distTable[last];
  float nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = mysqrtf(lastDist2) / g_phaseDistanceProj[phase];
  float nextDist = mysqrtf(nextDist2) / g_phaseDistanceProj[next];
  //RavlDebug("Last:%f  Next:%f ",lastDist,nextDist);
  angle -= (nextDist-lastDist)/(nextDist + lastDist);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle;
}

float hallToAngleDot2(uint16_t *sensors)
{
  float distTable[12];
  int phase = 0;
  float norm[3];
  norm[0] = (float) sensors[0] - originOffset;
  norm[1] = (float) sensors[1] - originOffset;
  norm[2] = (float) sensors[2] - originOffset;
  float mag = mysqrtf(sqr(norm[0]) + sqr(norm[1]) + sqr(norm[2]));
  for(int j = 0;j < 3;j++)
    norm[j] /= mag;

  //RavlDebug("Vec: %f %f %f",norm[0],norm[1],norm[2]);

  //mag = mysqrtf(sqr(g_phaseAngles[0][0]) + sqr(g_phaseAngles[0][1]) + sqr(g_phaseAngles[0][2]));

  float maxCorr = ((g_phaseAnglesNormOrg[0][0]) * norm[0]) +
                  ((g_phaseAnglesNormOrg[0][1]) * norm[1]) +
                  ((g_phaseAnglesNormOrg[0][2]) * norm[2]);

  distTable[0] = maxCorr;

  for(int i = 1;i < 12;i++) {
    //mag = mysqrtf(sqr(g_phaseAngles[i][0]) + sqr(g_phaseAngles[i][1]) + sqr(g_phaseAngles[i][2]));
    float corr = ((g_phaseAnglesNormOrg[i][0]) * norm[0]) +
                  ((g_phaseAnglesNormOrg[i][1]) * norm[1]) +
                  ((g_phaseAnglesNormOrg[i][2]) * norm[2]);
    distTable[i] = corr;
    //RavlDebug("Corr:%f ",corr);
    if(corr > maxCorr) {
      phase = i;
      maxCorr = corr;
    }
  }
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(next > 11) next = 0;
  float lastDist2 = distTable[last];
  float nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = maxCorr - lastDist2;
  float nextDist = maxCorr - nextDist2;
  //Average error:0.007239  Abs:0.207922 Mag:0.263140
  float corr = (nextDist-lastDist)/(nextDist + lastDist);
  angle -= corr;
  //RavlDebug("Last:%f  Max:%f Next:%f Corr:%f ",lastDist,maxCorr,nextDist,corr);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle;
}



float hallToAngleDot(uint16_t *sensors)
{
  float distTable[12];
  int phase = 0;
  float norm[3];
  norm[0] = (float) sensors[0] - hallMean[0];
  norm[1] = (float) sensors[1] - hallMean[1];
  norm[2] = (float) sensors[2] - hallMean[2];
  float mag = mysqrtf(sqr(norm[0]) + sqr(norm[1]) + sqr(norm[2]));
  for(int j = 0;j < 3;j++)
    norm[j] /= mag;

  //RavlDebug("Vec: %f %f %f",norm[0],norm[1],norm[2]);

  float maxCorr = (g_phaseAnglesNorm[0][0] * norm[0]) +
                  (g_phaseAnglesNorm[0][1] * norm[1]) +
                  (g_phaseAnglesNorm[0][2] * norm[2]);

  distTable[0] = maxCorr;

  for(int i = 1;i < 12;i++) {
    float corr = (g_phaseAnglesNorm[i][0] * norm[0]) +
                  (g_phaseAnglesNorm[i][1] * norm[1]) +
                  (g_phaseAnglesNorm[i][2] * norm[2]);
    distTable[i] = corr;
    //RavlDebug("Corr:%f ",corr);
    if(corr > maxCorr) {
      phase = i;
      maxCorr = corr;
    }
  }
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(next > 11) next = 0;
  float lastDist2 = distTable[last];
  float nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = maxCorr - lastDist2;
  float nextDist = maxCorr - nextDist2;
  //RavlDebug("Last:%f  Max:%f Next:%f ",lastDist,maxCorr,nextDist);
  angle -= (nextDist-lastDist)/(nextDist + lastDist);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle;
}


float hallToAngleNorm(uint16_t *sensors)
{
  float distTable[12];
  int phase = 0;
  float norm[3];
  norm[0] = (float) sensors[0] - hallMean[0];
  norm[1] = (float) sensors[1] - hallMean[1];
  norm[2] = (float) sensors[2] - hallMean[2];
  float mag = mysqrtf(sqr(norm[0]) + sqr(norm[1]) + sqr(norm[2]));
  for(int j = 0;j < 3;j++)
    norm[j] /= mag;

  //RavlDebug("Vec: %f %f %f",norm[0],norm[1],norm[2]);

  float minDist = sqr(g_phaseAnglesNorm[0][0] - norm[0]) +
                sqr(g_phaseAnglesNorm[0][1] - norm[1]) +
                sqr(g_phaseAnglesNorm[0][2] - norm[2]);

  distTable[0] = minDist;

  for(int i = 1;i < 12;i++) {
    float dist = sqr(g_phaseAnglesNorm[i][0] - norm[0]) +
                  sqr(g_phaseAnglesNorm[i][1] - norm[1]) +
                  sqr(g_phaseAnglesNorm[i][2] - norm[2]);
    distTable[i] = dist;
    //RavlDebug("Dist:%f ",dist);
    if(dist < minDist) {
      phase = i;
      minDist = dist;
    }
  }
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(next > 11) next = 0;
  float lastDist2 = distTable[last];
  float nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = mysqrtf(lastDist2) / g_phaseDistanceNorm[phase];
  float nextDist = mysqrtf(nextDist2) / g_phaseDistanceNorm[next];
  //RavlDebug("Last:%f  Next:%f ",lastDist,nextDist);
  angle -= (nextDist-lastDist)/(nextDist + lastDist);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle;
}


float hallToAngle(uint16_t *sensors)
{
  int distTable[12];
  int phase = 0;
  int minDist = sqr(g_phaseAngles[0][0] - sensors[0]) +
                sqr(g_phaseAngles[0][1] - sensors[1]) +
                sqr(g_phaseAngles[0][2] - sensors[2]);
  distTable[0] = minDist;

  for(int i = 1;i < 12;i++) {
    int dist = sqr(g_phaseAngles[i][0] - sensors[0]) +
                  sqr(g_phaseAngles[i][1] - sensors[1]) +
                  sqr(g_phaseAngles[i][2] - sensors[2]);
    distTable[i] = dist;
    if(dist < minDist) {
      phase = i;
      minDist = dist;
    }
  }
  int last = phase - 1;
  if(last < 0) last = 11;
  int next = phase + 1;
  if(next > 11) next = 0;
  int lastDist2 = distTable[last];
  int nextDist2 = distTable[next];
  float angle = phase * 2.0;
  float lastDist = mysqrtf(lastDist2) / g_phaseDistance[phase];
  float nextDist = mysqrtf(nextDist2) / g_phaseDistance[next];
  //RavlDebug("Last:%f  Next:%f ",lastDist,nextDist);
  angle -= (nextDist-lastDist)/(nextDist + lastDist);
  if(angle < 0.0) angle += 24.0;
  if(angle > 24.0) angle -= 24.0;
  return angle;
  //return angle * M_PI * 2.0 / 24.0;
}




