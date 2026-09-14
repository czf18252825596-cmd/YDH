/*******************************************************************************************************/
/*                                                头文件                                               */
/*******************************************************************************************************/
#include "ULink_Core.h"


/*******************************************************************************************************/
/*                                              内部函数声明                                            */
/*******************************************************************************************************/
static uint16_t ULink_CRC16(uint8_t *pBuf, uint16_t len);


/*******************************************************************************************************/
/*                                               内部函数定义                                           */
/*******************************************************************************************************/


/**
 * @brief IAPLink CRC16校验
 * @param pBuf  校验数据
 * @param len   数据长度
 * @return uint16_t  
 */
static uint16_t ULink_CRC16(uint8_t *pBuf, uint16_t len)
{
    uint16_t tmp = 0, CRC16 = 0xffff;

    for (uint16_t i = 0; i < len; i++)
    {
        CRC16 = *pBuf ^ CRC16;
        for (uint16_t j = 0; j < 8; j++)
        {
            tmp   = CRC16 & 0x0001;
            CRC16 = CRC16 >> 1;

            if (tmp)
                CRC16 = CRC16 ^ 0xA001;
        }
        pBuf++;
    }

    return CRC16;
}


/**
 * @brief IAPLink 应答
 * @param pIAP  IAPLink_t
 * @param cmd   指令
 * @return true  
 * @return false  
 */
bool ULink_EncodeSend(ULink_t *pU, uint8_t DEVICE_ID, uint8_t MS, uint8_t PACKET_ID, uint8_t *pData, uint16_t len)
{
	bool state = false;
	
	uint8_t buf[1024+256];
	uint16_t buf_len = 0;
	
	pU->Tx.HEAD = 0x7e;
	memcpy(&buf[0], &pU->Tx.HEAD, sizeof(pU->Tx.HEAD));
	
	pU->Tx.DEVICE_ID = DEVICE_ID;
	memcpy(&buf[1], &pU->Tx.DEVICE_ID, sizeof(pU->Tx.DEVICE_ID));
	
	pU->Tx.MS = MS;
	memcpy(&buf[2], &pU->Tx.MS, sizeof(pU->Tx.MS));
	
	pU->Tx.PACKET_ID = PACKET_ID;
	memcpy(&buf[3], &pU->Tx.PACKET_ID, sizeof(pU->Tx.PACKET_ID));
	
	pU->Tx.LENGTH = len;
	memcpy(&buf[4], &pU->Tx.LENGTH, sizeof(pU->Tx.LENGTH));
	
	if(pData != NULL)
	{
		pU->Tx.pDATA = pData;
		memcpy(&buf[6], pU->Tx.pDATA, pU->Tx.LENGTH);
	}
	
	pU->Tx.CRC16 = ULink_CRC16(buf, len+6);
	memcpy(&buf[6 + pU->Tx.LENGTH], &pU->Tx.CRC16, sizeof(pU->Tx.CRC16));
	
	buf_len = 8 + pU->Tx.LENGTH;
	
	state = pU->SendBuffer(pU->send_context, buf, buf_len);
	
	return state;
}

bool ULink_ReceiveDecode(ULink_t *pU, uint8_t *pDEVICE_ID, uint8_t *pMS, uint8_t *pPACKET_ID, uint8_t **pData, uint16_t *pLen)
{	
	bool state = false;
	
	uint8_t buf[1024+256] = {0};
	uint16_t buf_len = 0;

	state = pU->RecBuffer(pU->recv_context, buf, &buf_len);
	if(state == true)
	{
		for(int i = 0; i<buf_len; i++)
		{
			memcpy(&pU->Rx.HEAD, &buf[i], sizeof(pU->Rx.HEAD)); 
			if(pU->Rx.HEAD == 0x7e)
			{
				memcpy(&pU->Rx.LENGTH, &buf[i+4], sizeof(pU->Rx.LENGTH)); 
				if(pU->Rx.LENGTH <= buf_len)
				{
					memcpy(&pU->Rx.CRC16, &buf[i+6+pU->Rx.LENGTH], sizeof(pU->Rx.CRC16)); 
					uint16_t CRC16 = ULink_CRC16(buf, pU->Rx.LENGTH+6);
					if(pU->Rx.CRC16 == CRC16)
					{
						pU->Rx.pDATA = pU->BUFFER;
						
						memcpy(&pU->Rx.DEVICE_ID, &buf[i + 1], sizeof(pU->Rx.DEVICE_ID));  
						memcpy(&pU->Rx.MS, 		  &buf[i + 2], sizeof(pU->Rx.MS));  
                        memcpy(&pU->Rx.PACKET_ID, &buf[i + 3], sizeof(pU->Rx.PACKET_ID));
						memcpy(pU->Rx.pDATA, 	  &buf[i + 6], pU->Rx.LENGTH);
						
						if(pDEVICE_ID != NULL)
							memcpy(pDEVICE_ID, &pU->Rx.DEVICE_ID, sizeof(pU->Rx.DEVICE_ID));
						if(pMS != NULL)
							memcpy(pMS, &pU->Rx.MS, sizeof(pU->Rx.MS));
						if(pPACKET_ID != NULL)
							memcpy(pPACKET_ID, &pU->Rx.PACKET_ID, sizeof(pU->Rx.PACKET_ID));
						if(pData != NULL)
							*pData = pU->Rx.pDATA;
						if(pLen != NULL)
							memcpy(pLen, &pU->Rx.LENGTH, sizeof(pU->Rx.LENGTH));
							
                        return true;	
					}
					else
						return false;	
				}
				else
					return false;
			}
		}				
	}
	
	return state;
}

