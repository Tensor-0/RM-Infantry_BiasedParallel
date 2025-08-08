#include "bsp_uart.h"
#include "usart.h"
#include "remote_control.h"
#include <stdio.h>
#include <stdarg.h>
#include "motor.h"
#include "Referee_System.h"

 void usart_printf(const char *fmt,...){
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
	  __attribute__((section (".AXI_SRAM"))) static uint8_t tx_buf[256] = {0};
    len = vsnprintf((char *)tx_buf,sizeof(tx_buf) ,(char*)fmt, ap);
    
    va_end(ap);

    HAL_UART_Transmit_DMA(&huart7,(uint8_t*)tx_buf, len);
}
 __attribute__((section (".AXI_SRAM")))  uint8_t Rx_Buf[16];

void Usart_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3){

 uint8_t *SendValue1_Pointer,*SendValue2_Pointer,*SendValue3_Pointer;

 SendValue1_Pointer = (uint8_t *)&SendValue1;
 SendValue2_Pointer = (uint8_t *)&SendValue2;
 SendValue3_Pointer = (uint8_t *)&SendValue3;

	
 Rx_Buf[0] =  *SendValue1_Pointer;
 Rx_Buf[1] =  *(SendValue1_Pointer + 1);
 Rx_Buf[2] =  *(SendValue1_Pointer + 2);
 Rx_Buf[3] =  *(SendValue1_Pointer + 3);
 Rx_Buf[4] =  *SendValue2_Pointer;
 Rx_Buf[5] =  *(SendValue2_Pointer + 1);
 Rx_Buf[6] =  *(SendValue2_Pointer + 2);
 Rx_Buf[7] =  *(SendValue2_Pointer + 3);
 Rx_Buf[8] =  *SendValue3_Pointer;
 Rx_Buf[9] =  *(SendValue3_Pointer + 1);
 Rx_Buf[10] = *(SendValue3_Pointer + 2);
 Rx_Buf[11] = *(SendValue3_Pointer + 3);
 Rx_Buf[12] =  0x00;
 Rx_Buf[13] =  0x00;
 Rx_Buf[14] =  0x80;
 Rx_Buf[15] =  0x7F;
 HAL_UART_Transmit_DMA(&huart7,Rx_Buf,sizeof(Rx_Buf));
	

}

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxXferSize    = DataLength;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
		
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

  /* Configure the destination memory Buffer address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);	
	
	
}

void BSP_USART_Init(void){

		
	USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Referee_MultiRx_Buf[0],(uint32_t *)Referee_MultiRx_Buf[1],Referee_System_Info_Max_Length*2);

	  
	USART_RxDMA_MultiBuffer_Init(&huart5,(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);



}

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
	
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
			{
					//Disable DMA 
					__HAL_DMA_DISABLE(huart->hdmarx);

					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
					/* reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

					if(Size == RC_FRAME_LENGTH)
					{
						SBUS_TO_RC(SBUS_MultiRx_Buf[0],&remote_ctrl);
					}
					
			}
			/* Current memory buffer used is Memory 1 */
			else
			{
					//Disable DMA 
					__HAL_DMA_DISABLE(huart->hdmarx);
				
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				
					/* reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

					if(Size == RC_FRAME_LENGTH)
					{
						SBUS_TO_RC(SBUS_MultiRx_Buf[1],&remote_ctrl);
					}
					
					
			}
			
}

static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
	{
		
					__HAL_DMA_DISABLE(huart->hdmarx);

					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
				

				  if(Size >= 10){
						
						Referee_Frame_Update(Referee_MultiRx_Buf[0]);
				
				        memset(Referee_MultiRx_Buf[0],0,Referee_System_Info_Max_Length);

				      __HAL_DMA_SET_COUNTER(huart->hdmarx,Referee_System_Info_Max_Length*2);
          }
					
					
	}
	else
	{
					__HAL_DMA_DISABLE(huart->hdmarx);
				
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				

		    if(Size >= 10){
        
			   Referee_Frame_Update(Referee_MultiRx_Buf[1]);
				
				 memset(Referee_MultiRx_Buf[1],0,Referee_System_Info_Max_Length);

				 __HAL_DMA_SET_COUNTER(huart->hdmarx,Referee_System_Info_Max_Length*2);
      }
					
					
	}
  



}

static void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


}

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	
	 if(huart == &huart5){
	
		  USER_USART5_RxHandler(huart,Size);
			
	} 

	 if(huart == &huart1){
	 
      USER_USART1_RxHandler(huart,Size);
			
	}
	
   huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}



