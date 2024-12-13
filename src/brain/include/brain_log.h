#pragma once

#include <string>
#include <rerun.hpp>

class Brain; // Forward declaration

using namespace std;

/**
 * Operations related to rerun logs are handled in this library.
 * If there are repetitive logs that need to be printed, the functions can be encapsulated within this class.
 */
/**
*与重运行日志相关的操作在这个库中处理。
* 如果有重复的日志需要打印，函数可以封装在这个类中。
*/
class BrainLog
{
public:
    BrainLog(Brain *argBrain);
    
    void prepare();

    void logStatics();

    void setTimeNow();

    void setTimeSeconds(double seconds);

    // Expose the same interface as rerun::RecordingStream
    /*C++17 标准中引入的折叠表达式（fold expression）
    允许创建可以接受任意数量类型参数
    template <typename... Ts>
    void print(Ts... args) 
    {
        // 使用参数包展开来打印每个参数
        (std::cout << ... << args) << '\n';
    }
    int main() 
    {
        print(1, 2.5, "Hello"); // 输出：1 2.5 Hello
        return 0;
    }*/
    template <typename... Ts>
    inline void log(string_view entity_path, const Ts &...archetypes_or_collections) const
    {
        if (enabled)
            rerunLog.log(entity_path, archetypes_or_collections...);
    }

    void logToScreen(string logPath, string text, u_int32_t color, double padding = 0.0);

    inline bool isEnabled()
    {
        return enabled;
    }

private:
    bool enabled;
    Brain *brain;
    rerun::RecordingStream rerunLog;
};