#ifndef INCLUDED_UTIL
#define INCLUDED_UTIL

#include <vector>

typedef double Real;

Real lerp( Real a, Real b, Real t );
Real correlation(std::vector<Real> x, std::vector<Real> y);

#endif //ndef INCLUDED_UTIL
