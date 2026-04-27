#ifndef __COM_DEBUG_H__
#define __COM_DEBUG_H__

#include "usart.h"
#include <stdio.h>
#include <string.h>

/* 串口调试日志总开关。 */
#define DEBUG_LOG_ENABLE 1

/* 从完整路径中提取文件名，便于缩短日志输出。 */
#define __FILE_NAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__))

#if DEBUG_LOG_ENABLE
/* 输出带文件名和行号前缀的调试日志。 */
#define debug_printf(format, ...) printf("%s:%d " format, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#else
/* 关闭日志时编译为空操作。 */
#define debug_printf(...) ((void)0)
#endif

#endif
