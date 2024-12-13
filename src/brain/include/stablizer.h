#pragma once

#include <vector>
#include <chrono>
#include <iterator>
#include <limits>

using namespace std;
/*���д��봴����һ������chr����ָ��std::chrono�����ռ䡣
std::chrono��C++11���Ժ�汾�������һ����׼�⣬���ڴ������ں�ʱ�䡣
ʹ��chr::seconds����std::chrono::seconds*/
namespace chr = std::chrono;
/*Stablizer ����һ���򵥵������ȶ�����
��ͨ���������ݵĴ洢ʱ�������ٶ��ڲ�����Ӱ�졣
��������������Ҫƽ�����ݻ���������ĳ�����
������ʵʱϵͳ�д����������ݡ�
ͨ���ṩ��ͬ�ķ�����ƽ��ֵ����Сֵ�����ֵ���������ȶ���ֵ��
������ṩ�����������Ӧ��ͬ�����ݴ�������*/
class Stablizer
{
public:
    Stablizer()
    {
        //�κγ���2������ݽ�����Ϊ�����ݲ����Ƴ���
        _timespan = chr::milliseconds(2000);
    };
    //�����û��������ݱ�����ʱ���ȣ��Ժ���Ϊ��λ
    void setTimespan(int msec)
    {
        _timespan = chr::milliseconds(msec);
    };
    //�˺�������һ���µ����ݵ� x ��������ӵ��ڲ��洢�С�
    // ͬʱ������¼��ǰʱ�䣬������ _removeOldData �������Ƴ�������
    void push(double x)
    {
        auto cur_time = chr::high_resolution_clock::now();
        _t.push_back(cur_time);
        _x.push_back(x);
        _removeOldData();
    };
    /*�˺�������ָ���ķ������㲢�����ȶ���ֵ��֧�ֵķ���������
    "mean"���������б������ݵ�ƽ��ֵ��
    "min"���ҵ����б������ݵ���Сֵ��
    "max"���ҵ����б������ݵ����ֵ�� ����ڲ��洢Ϊ�ջ򷽷���Ч������0��*/
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
    //����ʱ��������ڴ洢���ݱ�����ʱ����
    chr::duration<int, milli> _timespan;
    //���ڴ洢����ֵ
    vector<double> _x;
    //���ڴ洢ÿ�����ݵ��ʱ�����
    vector<chr::high_resolution_clock::time_point> _t;
    //�˺����Ƴ����г��� _timespan ʱ�䷶Χ�ľ����ݡ�
    // ��ͨ���Ƚϵ�ǰʱ���ÿ�����ݵ��ʱ�����������Щ����Ӧ�ñ��Ƴ���
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