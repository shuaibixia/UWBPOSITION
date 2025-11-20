// 注释掉 #define WIN32 (1) 这一行 移除Windows强制定义
#ifndef __common_h__
#define __common_h__

// #include <iostream>
#include <stdio.h>

// #define    WIN32    (1)

#define    _WINSOCK_DEPRECATED_NO_WARNINGS    (2)

#define    HEADER_LEN       (95)
// 如歌网络协议  
#define    HUMAN_LEN        (52) 

#define    HEADER_1         (85) 
#define    HEADER_2         (170) 
#define    HEADER_3         (1) 

#define    TAG_1            (5) 
#define    TAG_2            (6) 
#define    BASE_3           (7) 
#define    BASE_4           (8) 

#define    DISTANCE_1       (9) 
#define    DISTANCE_2       (10) 
#define    DISTANCE_3       (11) 
#define    DISTANCE_4       (12) 

#define    SING_PACKAGE_LEN (95) 

typedef unsigned char      uint8_t;

#endif
