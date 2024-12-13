#pragma once
//自定义打印信息
#include <string>

using namespace std;

/* ------------------------------------ ANSI COLOR CODES ------------------------------------ */

// Color code
//定义ANSI颜色代码宏：这些宏允许你通过特定的转义序列来改变终端中文本的颜色
#define BLACK_CODE "\033[0;30m"
#define RED_CODE "\033[0;31m"
#define GREEN_CODE "\033[0;32m"
#define YELLOW_CODE "\033[0;33m"
#define BLUE_CODE "\033[0;34m"
#define MAGENTA_CODE "\033[0;35m"
#define CYAN_CODE "\033[0;36m"
#define WHITE_CODE "\033[0;37m"

// Bold versions
#define BOLD_BLACK_CODE "\033[1;30m"
#define BOLD_RED_CODE "\033[1;31m"
#define BOLD_GREEN_CODE "\033[1;32m"
#define BOLD_YELLOW_CODE "\033[1;33m"
#define BOLD_BLUE_CODE "\033[1;34m"
#define BOLD_MAGENTA_CODE "\033[1;35m"
#define BOLD_CYAN_CODE "\033[1;36m"
#define BOLD_WHITE_CODE "\033[1;37m"

// Background colors
#define BLACK_BG_CODE "\033[0;40m"
#define RED_BG_CODE "\033[0;41m"
#define GREEN_BG_CODE "\033[0;42m"
#define YELLOW_BG_CODE "\033[0;43m"
#define BLUE_BG_CODE "\033[0;44m"
#define MAGENTA_BG_CODE "\033[0;45m"
#define CYAN_BG_CODE "\033[0;46m"
#define WHITE_BG_CODE "\033[0;47m"

// Reset code
//将文本颜色和样式重置为默认值
#define RESET_CODE "\033[0m"

/*C printf style string formatter
inline内联，减少资源耗损
 （如果一些函数被频繁调用，
 不断地有函数入栈，即函数栈，
 会造成栈空间或栈内存的大量消耗）*/
//字符串格式化
//string greeting = format("Hello, %s!", "World");
inline string format(const char *str, ...)
{
    char buf[1024];
    va_list args;
    va_start(args, str);
    vsprintf(buf, str, args);
    va_end(args);
    return string(buf);
}

// Print the string str enclosed in top and bottom borders, and return the input string itself
/*以美化的方式打印信息，特别适合于日志记录、错误消息或任何需要突出显示的文本。
通过添加颜色和边框，它可以提高文本的可读性和视觉吸引力
prettyPrint(myString, "My Title", BOLD_MAGENTA_CODE);
==========================================================
My Title
==========================================================
Hello, World!
==========================================================*/
inline string prettyPrint(const string &str, const string &title = "", const string &colorCode = "", int borderLength = 70, char borderChar = '=')
{
    int headerHalfLength = floor((borderLength - title.length() - 2) / 2);
    if (headerHalfLength < 0)
        headerHalfLength = 0;
    string header = string(headerHalfLength, borderChar);
    string footer = string(borderLength, borderChar);
    cout << colorCode
         << header << " " << title << " " << header << "\n\n"
         << str << "\n\n"
         << footer 
         << RESET_CODE << "\n" << endl;
    return str;
}

// Print the string str enclosed in a prominent red block labeled "ERROR", and return the input string itself
/*prettyPrint 函数的一个便捷封装。
prtErr 函数专门用于打印错误信息，
并且会自动地将这些信息格式化为带有 "ERROR" 标题和红色字体的样式。*/
inline string prtErr(const string &str)
{
    return prettyPrint(str, "ERROR", RED_CODE);
}

// Print the string str enclosed in a prominent block labeled "DEBUG", and return the input string itself
inline string prtDebug(const string &str)
{
    return prettyPrint(str, "DEBUG", CYAN_CODE);
}

// Print the string str enclosed in a prominent block labeled "DEBUG", and return the input string itself.
inline string prtWarn(const string &str)
{
    return prettyPrint(str, "WARN", YELLOW_CODE);
}
