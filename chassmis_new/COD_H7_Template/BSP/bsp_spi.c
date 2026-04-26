#include "bsp_spi.h"
#include "spi.h"

//__attribute__((section (".RAM_D1")))  uint8_t Rx_Data;
//__attribute__((section (".RAM_D1")))  uint8_t Tx_Data;



uint8_t BMI088_Read_Write_Byte(uint8_t Tx_Data)
{
    //Tx_Data = Tx;
    uint8_t Rx_Data;
		HAL_SPI_TransmitReceive(&hspi2,&Tx_Data,&Rx_Data,1,100);
   
//	  while(__HAL_DMA_GET_COUNTER(hspi2.hdmarx)!=0){
//		
//		
//		}
	
    return Rx_Data;
}
//__attribute__((section (".RAM_D1")))  uint8_t Rx_Data = 0 ;
//__attribute__((section (".RAM_D1")))  uint8_t Tx = 0;
//uint8_t Rx = 0 ;
//uint8_t BMI088_Read_Write_Byte(uint8_t Tx_Data)
//{
//	 
////    Tx = Tx_Data; 
////	  HAL_SPI_Transmit_DMA(&hspi2,&Tx,1);
////	  HAL_SPI_Transmit_DMA(&hspi2,&Rx_Data,1);
////	
////	  Rx = Rx_Data;
////	  return Rx;
////}
//    //uint8_t Rx_Data = 0;
////    Tx = Tx_Data;
//		HAL_SPI_TransmitReceive_DMA(&hspi2,&Tx,&Rx_Data,1);
//	  while(__HAL_DMA_GET_COUNTER(hspi2.hdmarx)!=0){
//		    
//		
//		}
//////	  
//////		
//		//return Rx_Data;
//    Rx =  Rx_Data;
//    return  Rx;
//	
////		HAL_SPI_TransmitReceive(&hspi2,&Tx_Data,&Rx_Data,1,100);
////    //Rx =  Rx_Data;
////    return  Rx_Data;
//	
//	
//}


//void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi) 
//{

//	
//	//HAL_SPI_TransmitReceive_DMA(&hspi2,&Tx,&Rx_Data,1);
//	
//	
//	
//}