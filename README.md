###Scripts Python 主程序

###Lib 各类模块，坐标变换、滤波器、

##Config 配置文件，anchor 坐标、飞行时的PID 参数

###Data. 日志、UWB 原始数据、测试数据 （不上传 留存在自己本地 

###Tests  测试脚本和单元测试（不上传 留存自己本地 
文件：net/net.cpp
修改：将 #include "../lib/ybase/conversion.h" 这一行
替换为：#include "../lib/ybase/typedef.h"
原因：这是最关键的一步。它彻底切断了项目对 define.h -> exception.h -> exception.cpp 的依赖链，同时仍然为 net.cpp 提供了所需的 uint32 和 uint64 类型定义。
文件：lib/common.h
修改：删除或注释掉 #define WIN32 (1) 这一行。
原因：移除这个 Windows 强制定义，CMakeLists.txt 中的 -D_LINUX_ 才能生效，从而激活 rcmIf.c 和 rcm.c 中正确的 Linux 代码。
文件：main.cpp
修改：在文件顶部添加 #include <unistd.h>。
修改：将 Sleep (200); 替换为 usleep(200 * 1000);。
原因：使用 Linux 兼容的 usleep 函数（单位为微秒）。
文件：lib/rcm.c
修改：删除或注释掉 #pragma comment (lib, "ws2_32.lib") 这一行。
原因：这是 Windows 特有的链接器指令，在 Linux 上会导致错误。