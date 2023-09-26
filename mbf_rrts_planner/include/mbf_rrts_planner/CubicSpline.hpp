#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include <vector>

struct SplineSet
{
    double a;
    double b;
    double c;
    double d;
    double x;
};

std::vector<SplineSet> spline(std::vector<double>& x, std::vector<double>& y);
#endif
