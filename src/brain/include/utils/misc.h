#pragma once

#include "chrono"
#include <string>
#include <random>
#include <sstream>

// Calculate how many milliseconds have passed since start_time
//这个函数可以用来测量代码执行的时间，例如在性能测试中。
// 它返回一个整数，表示自 start_time 以来经过的毫秒数。
inline int msecsSince(std::chrono::high_resolution_clock::time_point start_time)
{
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}