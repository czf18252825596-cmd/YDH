#ifndef __ULINK_H
#define __ULINK_H

#ifdef __cplusplus
extern "C" {
#endif
	
/*******************************************************************************************************/
/*                                                头文件                                               */
/*******************************************************************************************************/
#include "ULink_Core.h"
#include "ULink_DataStruct.h"


/*******************************************************************************************************/
/*                                              外部函数声明                                            */
/*******************************************************************************************************/
void ULink_RegSendFunc(ULink_t *pU, bool (*buffer_send)(void* context, uint8_t *pBuf, uint16_t len), void* context);
void ULink_RegReciveFunc(ULink_t *pU, bool (*buffer_rec)(void* context, uint8_t *pBuf, uint16_t *pLen), void* context);
bool ULink_Send(ULink_t *pU, uint8_t DEVICE_ID, uint8_t MS, uint8_t PACKET_ID, uint8_t *pData, uint16_t len);
bool ULink_Receive(ULink_t *pU, uint8_t *pDEVICE_ID, uint8_t *pMS, uint8_t *pPACKET_ID, uint8_t **pData, uint16_t *pLen);

#ifdef __cplusplus
}
#endif

#endif
