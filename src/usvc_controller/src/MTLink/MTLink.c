#include "MTLink.h"


/**
  * @funNm  
  * @brief  
  * @param	
  * @retval 
*/ 
void MTLink_RegSendFunc(MTLink_t *pMT, bool (*buffer_send)(void* context, uint8_t *pBuf, uint16_t len), void* context)
{
    if (buffer_send != NULL)
    {
        pMT->SendBuffer = buffer_send;
        pMT->send_context = context;
    }
}


void MTLink_RegReciveFunc(MTLink_t *pMT, bool (*buffer_rec)(void* context, uint8_t *pBuf, uint16_t *pLen), void* context)
{
    if (buffer_rec != NULL)
    {
        pMT->RecBuffer = buffer_rec;
        pMT->recv_context = context;
    }
}

/**
  * @funNm  
  * @brief  
  * @param	
  * @retval 
*/
bool MTLink_Send(MTLink_t *pMT, uint8_t SID, uint8_t DID, uint16_t obj, uint8_t *pBuf, uint16_t len)
{
	bool state = false;
	
	state = MTLink_MsgSend(pMT, SID, DID, obj, pBuf, len);
	
	return state;
}



/**
  * @funNm  
  * @brief  
  * @param	
  * @retval 
*/
bool MTLink_Receive(MTLink_t *pMT, uint8_t *pSID, uint8_t *pDID, uint16_t *pObj, uint8_t **pBuf, uint16_t *pLen)
{
	bool state = false;
	
	state = MTLink_MsgReceive(pMT, pSID, pDID, pObj, pBuf, pLen);
	
	return state;
}




