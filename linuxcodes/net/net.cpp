
#include "net.h"
#include "../lib/common.h"
// #include "../lib/ybase/conversion.h"
// 这是为了替换掉 conversation.h里面的文件选择 typedef.h中产生同样效果的的类型定义 
#include "../lib/ybase/typedef.h" 
#include "../csv/log.h"
// header only include 用到的.h 就不用cmakelist 里面一个个添加了 
#include "../lib/ybase/conversion.h"
#include "../lib/ybase/buffer.h"

void    netMain()
{
    unsigned char  data [1000];
    unsigned len  { 1000 };

    HumaticNet();

    netRecv   (data, len);
    // printf    ("len: %d\n", len);
    HumaParse (data, len);
    // netParse(data, len);
    return;
}

void    netInit()
{
    // char bufIP [100] { "192.168.0.201" };
    // 如歌网络 
    char bufIP [100] { "192.168.1.200" };

    rcmIfInit (rcmIfIp, bufIP);
    return;
}

void    netFree()
{
    rcmIfClose();
    return;
}

void    netCmd (char* cmd, unsigned int len)
{
    rcmIfSendPacket (cmd, len);
}

void    netRecv (unsigned char* buf, unsigned int& len)
{
    len = rcmIfGetPacket (buf, len);
}
// 暂时没有用到这个netparse 函数 但是又缺少 ylib 库里面具体的 cpp 文件 连接器导致编译不通过
// void    netParse(unsigned char* buf, unsigned int len )
// {
//     int cntLng { 0 };
//     int cntId  { 0 };

//     uint32 dist   { 0 };
//     std::string tagId  { "" };
//     std::string baseId { "" };
//     const char* tagPtr { nullptr };

//     std::stringstream traceLog;

//     if (len != HEADER_LEN)
//         return;

//     for (int i = 0; i < len; i++)
//     {
//         if ((buf[i]     == HEADER_1) && 
//             (buf[i + 1] == HEADER_2) && 
//             (buf[i + 2] == HEADER_3))
//         {
//             cntLng = buf[i + 3];
//             // 包序号 
//             cntId  = buf[i + 4];
//             // 处理一个测距包 
//             tagId  = std::to_string (buf[TAG_2] );
//             tagId += std::to_string (buf[TAG_1] );   

//             baseId = std::to_string (buf[BASE_4]);
//             baseId+= std::to_string (buf[BASE_3]);

//             tagPtr = (const char*)&(buf[DISTANCE_1]);    
//             ylib::bytes::to_uint (dist, tagPtr);
//             i += SING_PACKAGE_LEN;
            
//             traceLog<< cntId  << "," << tagId << "," 
//                     << baseId << "," << dist  << ",";

//             printf ("%s\n", (traceLog.str ().c_str()));
//             BOOST_LOG_TRIVIAL (trace) << (traceLog.str ().c_str());  
//         }
//     }
//     return;
// }
// 如歌发送网络包 
void    HumaticNet()
{
    char cm [100];
    unsigned int m { 12 };
    // 00 03 00 16 00 
    // 00 00 ca 00 00 
    // 00 00 
    cm[0] = 0;
    cm[1] = 3;
    cm[2] = 0;
    cm[3] = 22;
    cm[4] = 0;
    
    cm[5] = 0;
    cm[6] = 0;
    cm[7] = 202;
    cm[8] = 0;
    cm[9] = 0;
    
    cm[10]= 0;
    cm[11]= 0;
    netCmd (cm, m);
}

void    HumaParse (unsigned char* buf, unsigned int len )
{
    int cntLng { 0 };
    int cntId  { 0 };

    uint64 dist   { 0 };

    std::string tagId  { "" };
    std::string baseId { "" };
    const char* tagPtr { nullptr };

    std::stringstream traceLog;

    if (len != HUMAN_LEN)
        return;

    dist += (buf [12] << 24);
    dist += (buf [13] << 16);
    dist += (buf [14] << 8 );
    dist += buf [15];
    // tagPtr = (const char*)&(buf [12]);
    // ylib::bytes::to_uint64 (dist, tagPtr);
    traceLog<< "distance = " << dist;
    printf ("%s\n", (traceLog.str ().c_str()));
    return;
}
