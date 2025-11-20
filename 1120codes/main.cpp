/*
    运行在 和利时平台上
    引用 boost 
 */ 
#include <unistd.h>
#include "./pos/posMain.h"
#include "./vel/velMain.h"
#include "./net/net.h"

#include "./csv/csv.h"

// void main()
int main()
{
    InitDFile();
    netInit  ();

    while (true)
    {
        netMain();
        posMain();
        velMain();
        // & 周期 200 ms
		// sleep (200); win下的 sleep 是秒 Linux 是微秒
        usleep(200 * 1000);
    }
    netFree();
    return 0 ;
}
