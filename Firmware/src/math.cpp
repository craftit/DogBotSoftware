// based on https://math.stackexchange.com/a/1105038/81278
#define MACRO_MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MACRO_MIN(x, y) (((x) < (y)) ? (x) : (y))

static float fast_atan2(float y, float x)
{
  // a := min (|x|, |y|) / max (|x|, |y|)
  float abs_y = fabsf(y);
  float abs_x = fabsf(x);
  float a = MACRO_MIN(abs_x, abs_y) / MACRO_MAX(abs_x, abs_y);
  //s := a * a
  float s = a * a;
  //r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
  float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  //if |y| > |x| then r := 1.57079637 - r
  if (abs_y > abs_x)
      r = 1.57079637f - r;
  // if x < 0 then r := 3.14159274 - r
  if (x < 0.0f)
      r = 3.14159274f - r;
  // if y < 0 then r := -r
  if (y < 0.0f)
      r = -r;

  return r;
}


static void FastSinCos(float x,float &sin,float &cos)
{
  if (x < -3.14159265)
      x += 6.28318531;
  else
  if (x >  3.14159265)
      x -= 6.28318531;

  //compute sine
  if (x < 0)
  {
      sin = 1.27323954 * x + .405284735 * x * x;

      if (sin < 0)
          sin = .225 * (sin *-sin - sin) + sin;
      else
          sin = .225 * (sin * sin - sin) + sin;
  }
  else
  {
      sin = 1.27323954 * x - 0.405284735 * x * x;

      if (sin < 0)
          sin = .225 * (sin *-sin - sin) + sin;
      else
          sin = .225 * (sin * sin - sin) + sin;
  }

  //compute cosine: sin(x + PI/2) = cos(x)
  x += 1.57079632;
  if (x >  3.14159265)
      x -= 6.28318531;

  if (x < 0)
  {
      cos = 1.27323954 * x + 0.405284735 * x * x;

      if (cos < 0)
          cos = .225 * (cos *-cos - cos) + cos;
      else
          cos = .225 * (cos * cos - cos) + cos;
  }
  else
  {
      cos = 1.27323954 * x - 0.405284735 * x * x;

      if (cos < 0)
          cos = .225 * (cos *-cos - cos) + cos;
      else
          cos = .225 * (cos * cos - cos) + cos;
  }
}
