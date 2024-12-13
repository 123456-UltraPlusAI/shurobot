#pragma once

#include <vector>
#include <chrono>
#include <iterator>
#include <limits>

using namespace std;
/*这行代码创建了一个别名chr，它指向std::chrono命名空间。
std::chrono是C++11及以后版本中引入的一个标准库，用于处理日期和时间。
使用chr::seconds代替std::chrono::seconds*/
namespace chr = std::chrono;
/*Stablizer 类是一个简单的数据稳定器，
它通过限制数据的存储时间来减少短期波动的影响。
这个类可以用于需要平滑数据或减少噪声的场景，
例如在实时系统中处理传感器数据。
通过提供不同的方法（平均值、最小值、最大值）来计算稳定的值，
这个类提供了灵活性以适应不同的数据处理需求。*/
class Stablizer
{
public:
    Stablizer()
    {
        //任何超过2秒的数据将被视为旧数据并被移除。
        _timespan = chr::milliseconds(2000);
    };
    //允许用户设置数据保留的时间跨度（以毫秒为单位
    void setTimespan(int msec)
    {
        _timespan = chr::milliseconds(msec);
    };
    //此函数接受一个新的数据点 x 并将其添加到内部存储中。
    // 同时，它记录当前时间，并调用 _removeOldData 函数来移除旧数据
    void push(double x)
    {
        auto cur_time = chr::high_resolution_clock::now();
        _t.push_back(cur_time);
        _x.push_back(x);
        _removeOldData();
    };
    /*此函数根据指定的方法计算并返回稳定的值。支持的方法包括：
    "mean"：计算所有保留数据的平均值。
    "min"：找到所有保留数据的最小值。
    "max"：找到所有保留数据的最大值。 如果内部存储为空或方法无效，返回0。*/
    double getStablized(string method)
    {
        _removeOldData();
        if (_x.size() == 0)
            return 0;

        if (method == "mean")
        {
            double sum = 0;
            for (double num : _x)
            {
                sum += num;
            }
            return sum / _x.size();
        }

        if (method == "min")
        {
            double min = std::numeric_limits<double>::max();
            for (double num : _x)
            {
                if (num < min)
                {
                    min = num;
                }
            }
            return min;
        }

        if (method == "max")
        {
            double max = -std::numeric_limits<double>::max();
            for (double num : _x)
            {
                if (num > max)
                {
                    max = num;
                }
            }
            return max;
        }

        return 0;
    };

private:
    //持续时间对象，用于存储数据保留的时间跨度
    chr::duration<int, milli> _timespan;
    //用于存储数据值
    vector<double> _x;
    //用于存储每个数据点的时间戳。
    vector<chr::high_resolution_clock::time_point> _t;
    //此函数移除所有超出 _timespan 时间范围的旧数据。
    // 它通过比较当前时间和每个数据点的时间戳来决定哪些数据应该被移除。
    void _removeOldData()
    {
        auto curTime = chr::high_resolution_clock::now();
        if (_t.size() == 0)
            return;

        int i;
        for (i = 0; i < _t.size(); i++)
        {
            if (_t[i] + _timespan > curTime)
                break;
        }
        _t.erase(_t.begin(), std::next(_t.begin(), i));
        _x.erase(_x.begin(), std::next(_x.begin(), i));
    }
};