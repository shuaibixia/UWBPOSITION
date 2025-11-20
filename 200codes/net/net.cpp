#include "net.h"
#include "../lib/common.h"
#include "../lib/ybase/typedef.h" 
#include "../csv/log.h"
#include "../lib/rcm.h"           
#include "../lib/rn.h"            
#include <string.h>             
#include <stdio.h>              
#include <stdlib.h>             // 用于 exit()

const int ANCHOR_ID = 200; 

// --- 配置模式下的核心函数 (只运行一次) ---
void    netInit()
{
    // *** [START: Node 200 配置逻辑] ***
    char bufIP [100] { "192.168.1.200" }; 
    const int TARGET_NODE_ID = ANCHOR_ID; 

    // 1. 初始化网络连接
    if (rcmIfInit (rcmIfIp, bufIP) != OK) {
        printf("FATAL ERROR: Failed to connect to Node %s. Check IP and connection.\n", bufIP);
        exit(1);
    }
    
    // 2. 设置操作模式为 RCM 模式
    rcmOpModeSet(RCRM_OPMODE_RCM);

    // 3. 配置 Node ID 和天线
    rcmConfiguration config;
    if (rcmConfigGet(&config) == OK)
    {
        config.nodeId = TARGET_NODE_ID; 
        config.persistFlag = 1;         
        config.antennaMode = RCRM_ANTENNAMODE_TXA_RXA; 
        
        if (rcmConfigSet(&config) == OK) {
             printf("SUCCESS: Node %d (IP %s) configured and saved to flash.\n", TARGET_NODE_ID, bufIP);
        } else {
             printf("ERROR: Failed to set Node %d configuration.\n", TARGET_NODE_ID);
        }
    } else {
        printf("ERROR: Failed to read existing configuration from Node %d.\n", TARGET_NODE_ID);
    }
    
    // 配置完成后立即退出
    exit(0); 
    // *** [END: Node 200 配置逻辑] ***
}

// --- 保持其他函数为空，以满足 net.h 的声明 ---
void    netMain() { return; }
void    netFree() { rcmIfClose(); return; }
void    netCmd (char* cmd, unsigned int len) { rcmIfSendPacket (cmd, len); }
void    netRecv (unsigned char* buf, unsigned int& len) { len = rcmIfGetPacket (buf, len); }
void    netParse(unsigned char* buf, unsigned int len ) { return; }
void    HumaticNet() { return; }
void    HumaParse (unsigned char* buf, unsigned int len ) { return; }