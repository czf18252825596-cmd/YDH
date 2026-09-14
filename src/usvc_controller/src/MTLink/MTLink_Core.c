#include "MTLink_Core.h"



static void MTLink_CRC(uint8_t *pBuf, uint16_t len, uint8_t *pCRC_H, uint8_t *pCRC_L)
{
	if(pCRC_H == NULL || pCRC_L == NULL)
		return;
	
	uint16_t tmp = 0, CRC16 = 0xffff;
		
	for (uint16_t i=0; i<len; i++)
	{
		CRC16 = *pBuf^CRC16;
		for (uint16_t j=0; j<8; j++)
		{
			tmp   = CRC16 & 0x0001;
			CRC16 = CRC16 >>1;
			
			if(tmp)
				CRC16 = CRC16 ^ 0xA001; 
		}
		pBuf++;
	}
	*pCRC_H = CRC16>>8;
	*pCRC_L = CRC16&0xff;		
}


bool MTLink_MsgSend(MTLink_t *pMT, uint8_t SID, uint8_t DID, uint16_t obj, uint8_t *pBuf, uint16_t len)
{
	bool state = false;
	
	uint8_t  buf[2048] = {0};
    uint16_t buf_len   = 0;
	
	pMT->Tx.STX = 0xFE;
	buf[0] = pMT->Tx.STX;
	
	if(len <= PACKMAXLEN) 
	{
		pMT->Tx.Len = len;
		memcpy(&buf[1], &pMT->Tx.Len, sizeof(pMT->Tx.Len));
		
		buf[3] = pMT->Tx.RPC;
		
		pMT->Tx.PSTATE.PSTATE_BIT.Res = 0; 
		pMT->Tx.PSTATE.PSTATE_BIT.ACK = 0; 
		pMT->Tx.PSTATE.PSTATE_BIT.SorR = 1; 
		buf[4] = pMT->Tx.PSTATE.PSTATE_BETY;
		
		buf[5] = pMT->Tx.CNT; 
		
		pMT->Tx.SID = SID; 
		buf[6] = pMT->Tx.SID;
		
		pMT->Tx.DID = DID; 
		buf[7] = pMT->Tx.DID;
		
		pMT->Tx.Object = obj;
		memcpy(&buf[8], &pMT->Tx.Object, sizeof(pMT->Tx.Object));
		
		if(pBuf != NULL)
		{
			pMT->Tx.pPayLoad = pBuf;
			memcpy(&buf[10], pMT->Tx.pPayLoad, pMT->Tx.Len);
		}
		
		pMT->Tx.CNT++; 
		buf[5] = pMT->Tx.CNT; 
		
		MTLink_CRC(buf, 10+pMT->Tx.Len, &pMT->Tx.CRCH, &pMT->Tx.CRCL); 
		
		buf[11+pMT->Tx.Len-1] = pMT->Tx.CRCL;
		buf[12+pMT->Tx.Len-1] = pMT->Tx.CRCH;
		
		buf_len = 12 + pMT->Tx.Len;
		
		if(pMT->SendBuffer == NULL) 
			return false;
		
		state = pMT->SendBuffer(pMT->send_context, buf, buf_len); 	
	}
	else
	{
		state= false;
	}

	return state;
}




bool MTLink_MsgReceive(MTLink_t *pMT, uint8_t *pSID, uint8_t *pDID, uint16_t *pObj, uint8_t **pBuf, uint16_t *pLen)
{
	bool state = false;
	
	uint8_t  buf[2048] = {0};
    uint16_t buf_len   = 0;

	if(pMT->RecBuffer == NULL) 
		return false;
	
	if (pMT->RecBuffer(pMT->recv_context, buf, &buf_len))  
	{
		for (uint16_t i = 0; i <= buf_len; i++)
        { 
			pMT->Rx.STX = buf[i]; 
			if(pMT->Rx.STX == 0xFE)
			{
				memcpy(&pMT->Rx.Len, &buf[i + 1], sizeof(pMT->Rx.Len)); 
				if(pMT->Rx.Len <= buf_len) 
				{
					MTLink_CRC(&buf[i], pMT->Rx.Len+10, &pMT->Rx.CRCH, &pMT->Rx.CRCL); 
					if(pMT->Rx.CRCL == buf[i+pMT->Rx.Len+10] && pMT->Rx.CRCH == buf[i+pMT->Rx.Len+11])  
					{
						memcpy(&pMT->Rx.RPC, 				&buf[i+3], sizeof(pMT->Rx.RPC));
						memcpy(&pMT->Rx.PSTATE.PSTATE_BETY, &buf[i+4], sizeof(pMT->Rx.PSTATE.PSTATE_BETY));
						memcpy(&pMT->Rx.CNT, 				&buf[i+5], sizeof(pMT->Rx.CNT));
						memcpy(&pMT->Rx.SID,		 		&buf[i+6], sizeof(pMT->Rx.SID));
						memcpy(&pMT->Rx.DID, 				&buf[i+7], sizeof(pMT->Rx.DID));
						memcpy(&pMT->Rx.Object, 			&buf[i+8], sizeof(pMT->Rx.Object));
						pMT->Rx.pPayLoad = &buf[i+10];
						
				
						if(pMT->Rx.PSTATE.PSTATE_BIT.Res == 1) 
						{	
							state = false;   
							break;
						}
						if(pMT->Rx.PSTATE.PSTATE_BIT.ACK == 1) 
						{
							state = false;  
							break;
						}
						if(pMT->Rx.PSTATE.PSTATE_BIT.SL == 1) 
						{
							state = false;  
							break;
						}
						else
						{	
							memcpy(pMT->SHORT_BUFFER, pMT->Rx.pPayLoad, pMT->Rx.Len);  
							
							if(pSID != NULL)
								memcpy(pSID, &pMT->Rx.SID, 		sizeof(pMT->Rx.SID));
							
							if(pDID != NULL)
								memcpy(pDID, &pMT->Rx.DID, 		sizeof(pMT->Rx.DID));
							
							if(pObj != NULL)
								memcpy(pObj, &pMT->Rx.Object, 	sizeof(pMT->Rx.Object));
							
							if(pBuf != NULL)
								*pBuf = pMT->SHORT_BUFFER;  
							
							if(pLen != NULL)
								memcpy(pLen, &pMT->Rx.Len, 		sizeof(pMT->Rx.Len));
							
							state = true;
							return true;
						}
					}
					else
					{
						state = false;
						break;
					}
				}
				else
				{
					state = false;
					break;
				}
			}
		}
	}
	
	return state;
}





