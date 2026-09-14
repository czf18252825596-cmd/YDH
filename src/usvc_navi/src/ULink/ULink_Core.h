#ifndef __ULINKCORE_H
#define __ULINKCORE_H

#ifdef __cplusplus
extern "C" {
#endif
	
/*******************************************************************************************************/
/*                                                头文件                                               */
/*******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>



/*******************************************************************************************************/
/*                                               类型定义                                               */
/*******************************************************************************************************/
#pragma pack(1)
typedef struct
{
    uint8_t HEAD;       
    uint8_t DEVICE_ID;  
	uint8_t MS;    
    uint8_t PACKET_ID;       
    uint16_t LENGTH;       //包数
    uint8_t *pDATA;  //负载数据
    uint16_t CRC16;     //校验
} ULink_Field_t;

typedef struct
{
	ULink_Field_t Tx;
	ULink_Field_t Rx;
	
    bool (*SendBuffer)(void* context, uint8_t *pBuf, uint16_t len);
    bool (*RecBuffer)(void* context, uint8_t *pBuf, uint16_t *pLen);
	
	void* send_context;  // 用于存储发送函数的上下文（如类实例）
    void* recv_context;  // 用于存储接收函数的上下文

    uint8_t BUFFER[1024];
	
} ULink_t;
#pragma pack()

bool ULink_EncodeSend(ULink_t *pU, uint8_t DEVICE_ID, uint8_t MS, uint8_t PACKET_ID, uint8_t *pData, uint16_t len);
bool ULink_ReceiveDecode(ULink_t *pU, uint8_t *pDEVICE_ID, uint8_t *pMS, uint8_t *pPACKET_ID, uint8_t **pData, uint16_t *pLen);


/*******************************************************************************************************/
/*                                              外部函数声明                                            */
/*******************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
