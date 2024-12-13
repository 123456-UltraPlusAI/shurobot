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
*����������־��صĲ�����������д���
* ������ظ�����־��Ҫ��ӡ���������Է�װ��������С�
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
    /*C++17 ��׼��������۵����ʽ��fold expression��
    ���������Խ��������������Ͳ���
    template <typename... Ts>
    void print(Ts... args) 
    {
        // ʹ�ò�����չ������ӡÿ������
        (std::cout << ... << args) << '\n';
    }
    int main() 
    {
        print(1, 2.5, "Hello"); // �����1 2.5 Hello
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