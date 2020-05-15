#ifndef ED_SENSOR_INTEGRATION_GENERIC_FUNCTIONS_H_
#define ED_SENSOR_INTEGRATION_GENERIC_FUNCTIONS_H_

#include <iostream>
#include <vector>

class Point
{

public:
    float x, y;

    Point ( double x_in, double y_in );
};

struct greater
{
    template<class T>
    bool operator()(T const &a, T const &b) const { return a < b; }
};

template<typename T>
void wrap2Interval ( T* alpha, T lowerBound, T upperBound ) // Template code must be implemented in the header file. 
{
    T delta = upperBound - lowerBound;

    if ( *alpha < lowerBound )
    {
        while ( *alpha < lowerBound )
        {
            *alpha += delta;
        }
    }
    else if ( *alpha >= upperBound )
    {
        while ( *alpha >= upperBound )
        {
            *alpha -= delta;
        }
    }
}

template <typename T>
void append(std::vector<T>& a, const std::vector<T>& b)
{
    a.reserve(a.size() + b.size());
    a.insert(a.end(), b.begin(), b.end());
}

template <typename T>
void append(std::vector<T>& a, const std::vector<T>& b, int bStart, int bEnd)
{
    a.reserve(a.size() + bEnd - bStart );
    a.insert(a.end(), b.begin() + bStart, b.begin() + bEnd);
}

#endif