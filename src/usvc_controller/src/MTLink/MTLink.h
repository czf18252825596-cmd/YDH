#ifndef __MTLINK_H
#define __MTLINK_H

#include "MTLink_Core.h"
#include "MTLink_DataStruct.h"


void MTLink_RegSendFunc(MTLink_t *pMT, bool (*buffer_send)(void* context, uint8_t *pBuf, uint16_t len), void* context);
void MTLink_RegReciveFunc(MTLink_t *pMT, bool (*buffer_rec)(void* context,uint8_t *pBuf, uint16_t *pLen), void* context);


bool MTLink_Send(MTLink_t *pMT, uint8_t SID, uint8_t DID, uint16_t obj, uint8_t *pBuf, uint16_t len);
bool MTLink_Receive(MTLink_t *pMT, uint8_t *pSID, uint8_t *pDID, uint16_t *pObj, uint8_t **pBuf, uint16_t *pLen);

#endif


