#ifndef _COMMON_INCLUDES_H_
#define _COMMON_INCLUDES_H_

#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

#include "types/vxTypes.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif

//Basic math and conversion convenience inline functions.

template<typename T>
inline int8_t signum(T const& val)
{
	return (val==0)?0:(val<0)?-1:1;
}

template<typename T>
inline T maximum(T const& lhs, T const& rhs)
{
    return (lhs > rhs) ? lhs : rhs;
}

template<typename T>
inline T minimum(T const& lhs, T const& rhs)
{
    return (lhs < rhs) ? lhs : rhs;
}

inline double MetersToInches(double meters)
{
	return meters * 100.0 / 2.54;
}

inline double InchesToMeters(double inches)
{
	return inches * 2.54 / 100.0;
}

inline double FeetToMeters(double feet)
{
    return InchesToMeters(12 * feet);
}

inline double MetersToFeet(double meters)
{
	return MetersToInches(meters) / 12.0;
}

inline double PoundsToKilograms(double pounds)
{
    return pounds / 2.20462262;
}

inline double DegreesToRadians(double degrees)
{
    return degrees * M_PI / 180;
}

inline double RadiansToDegrees(double radians)
{
    return radians * 180 / M_PI;
}

#endif
