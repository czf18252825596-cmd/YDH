#ifndef __MTLINKCODEC_H
#define __MTLINKCODEC_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#ifndef NULL
	#define NULL 0
#endif


#define PACKMAXLEN 1024  


#pragma pack(1)
typedef struct
{
	uint8_t  STX;
	uint16_t Len;
	uint8_t  RPC;
	union
	{
		uint8_t  PSTATE_BETY;
		struct
		{
			uint8_t SorR:1;
			uint8_t ACK:1;
			uint8_t ES:2;
			uint8_t SL:1;
			uint8_t unused:2;
			uint8_t Res:1;
		}PSTATE_BIT;
	}PSTATE;
	uint8_t  CNT;
	uint8_t  SID;
	uint8_t  DID;
	uint16_t  Object;
	uint8_t  *pPayLoad;
	uint8_t  CRCL;
	uint8_t  CRCH;
}MTLink_Field_t;

typedef struct
{
	MTLink_Field_t Tx;  
	MTLink_Field_t Rx;  
	
	bool (*SendBuffer)(void* context, uint8_t *pBuf, uint16_t len);
    bool (*RecBuffer)(void* context, uint8_t *pBuf, uint16_t *pLen);
	
	void* send_context;  // 用于存储发送函数的上下文（如类实例）
    void* recv_context;  // 用于存储接收函数的上下文

	uint8_t SHORT_BUFFER[1024];

}MTLink_t;
#pragma pack()


bool MTLink_MsgSend(MTLink_t *pMT, uint8_t SID, uint8_t DID, uint16_t obj, uint8_t *pBuf, uint16_t len);
bool MTLink_MsgReceive(MTLink_t *pMT, uint8_t *pSID, uint8_t *pDID, uint16_t *pObj, uint8_t **pBuf, uint16_t *pLen);


#endif



