/*
 * @Author: Yangxl
 * @Date: 2019-05-29 15:26:03
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2019-09-16 03:00:06
 * @Description: 运行日志头文件
 */

#ifndef md_syslog__h
#define md_syslog__h

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>

#define DEBUG
#ifdef DEBUG
// logMsg信息级别
enum
{
    logInfo = 0,
    logWarn,
    logErr,
    logFatal
};
#define logMsg(debug_level, format, ...)                                                                   \
    do                                                                                                     \
    {                                                                                                      \
        switch (debug_level)                                                                               \
        {                                                                                                  \
        case 0:                                                                                            \
            printf("[Info ] " format "\r\n", ##__VA_ARGS__);                                               \
            break;                                                                                         \
        case 1:                                                                                            \
            printf("[Warn ] %s: " format "\r\n", __FUNCTION__, ##__VA_ARGS__);                             \
            break;                                                                                         \
        case 2:                                                                                            \
            printf("[Error] %s, %d: " format "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);               \
            break;                                                                                         \
        case 3:                                                                                            \
            printf("[Fatal] %s, %s, %d: " format "\r\n", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            break;                                                                                         \
        default:                                                                                           \
            break;                                                                                         \
        }                                                                                                  \
    } while (0)
#else
#define logMsg(debug_level, format, ...)
#endif

#ifdef __cplusplus
}
#endif

#endif