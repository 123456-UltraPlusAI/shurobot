#pragma once
//数值变换，计算方法
#include <cmath>
#include <vector>
#include "../types.h"

using namespace std;

// degrees to radians
//角度变换
inline double deg2rad(double deg)
{
    return deg / 180.0 * M_PI;
}

// radians to degrees
inline double rad2deg(double rad)
{
    return rad / M_PI * 180.0;
}

// arithmetic mean
//这个有必要吗？？？
inline double mean(double x, double y)
{
    return (x + y) / 2;
}

// truncate the number to a range
//用于将一个数值 x 限制在指定的范围内。
// 如果 x 超出了这个范围，它将被调整到范围的上界或下界
inline double cap(double x, double upper_limit, double lower_limit)
{
    return max(min(x, upper_limit), lower_limit);
}

// Calculate the L2 norm (the square root of the sum of the squares of two numbers).
inline double norm(double x, double y)
{
    return sqrt(x * x + y * y);
}
//函数重载，double,vector
// Calculate the L2 norm (the square root of the sum of the squares of two numbers).
inline double norm(vector<double> v)
{
    return sqrt(v[0] * v[0] + v[1] * v[1]);
}

// Convert an angle to the range of [-M_PI, M_PI).
//将周期弧度等价为[-pi,pi]
inline double toPInPI(double theta)
{
    int n = static_cast<int>(fabs(theta / 2 / M_PI)) + 1;
    return fmod(theta + M_PI + 2 * n * M_PI, 2 * M_PI) - M_PI;
}

// In any Cartesian coordinate system, calculate the angle θ (in radians) between a vector v 
// and the x-axis, with the range (-M_PI, M_PI).
//计算向量 V与x轴之间的角度 （以弧度为单位），其范围是 (-pi, pi) 。
inline double thetaToX(vector<double> v)
{
    vector<double> x = {1, 0};
    double ang = atan2(v[1], v[0]);
    return toPInPI(ang);
}

// Transform a point from coordinate system 0 to coordinate system 1, where coordinate system 1 
// is rotated by an angle θ relative to coordinate system 0.
//????
inline Point2D transform(Point2D p0, double theta)
{
    Point2D p1;
    p1.x = p0.x * cos(theta) + p0.y * sin(theta);
    p1.y = -p0.x * sin(theta) + p0.y * cos(theta);
    return p1;
}

/**
 * @brief Transform a Pose (xs, ys, thetas) from source coordinate system (s) to target coordinate system (t).
 *        The source coordinate system's origin (xst, yst, thetast) is represented in the target coordinate system.
 *
 * @param xs, ys, thetas Pose (position and orientation) in the source coordinate system (s), with theta in radians.
 * @param xst, yst, thetast Position and orientation of the source coordinate system's origin in the target coordinate system (t), with theta in radians.
 * @param xt, yt, thetat Output the Pose (position and orientation) in the target coordinate system (t), with theta in radians.
 */

inline void transCoord(const double &xs, const double &ys, const double &thetas, const double &xst, const double &yst, const double &thetast, double &xt, double &yt, double &thetat)
{
    thetat = toPInPI(thetas + thetast);
    //旋转和平移，变换矩阵，先旋转再平移
    xt = xst + xs * cos(thetast) - ys * sin(thetast);
    yt = yst + xs * sin(thetast) + ys * cos(thetast);
}