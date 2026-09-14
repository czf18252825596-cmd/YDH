#include "ULink.h"


void ULink_RegSendFunc(ULink_t *pU, bool (*buffer_send)(void* context, uint8_t *pBuf, uint16_t len), void* context)
{
    if (buffer_send != NULL)
    {
        pU->SendBuffer = buffer_send;
		pU->send_context = context;
    }
}


void ULink_RegReciveFunc(ULink_t *pU, bool (*buffer_rec)(void* context, uint8_t *pBuf, uint16_t *pLen), void* context)
{
    if (buffer_rec != NULL)
    {
        pU->RecBuffer = buffer_rec;
		pU->recv_context = context;
    }
}



bool ULink_Send(ULink_t *pU, uint8_t DEVICE_ID, uint8_t MS, uint8_t PACKET_ID, uint8_t *pData, uint16_t len)
{
    bool    state = false;
    uint8_t cnt   = 0;

    do
    {
        state = ULink_EncodeSend(pU, DEVICE_ID, MS, PACKET_ID, pData, len);
        cnt++;
    } while (state == false && cnt < 3);

    return state;
}


bool ULink_Receive(ULink_t *pU, uint8_t *pDEVICE_ID, uint8_t *pMS, uint8_t *pPACKET_ID, uint8_t **pData, uint16_t *pLen)
{
    return ULink_ReceiveDecode(pU, pDEVICE_ID, pMS, pPACKET_ID, &(*pData), pLen);
}

