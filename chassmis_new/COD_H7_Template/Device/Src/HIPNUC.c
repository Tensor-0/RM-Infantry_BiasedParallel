#include "HIPNUC.h"
#include "Referee_System.h"

__attribute__((section (".AXI_SRAM"))) uint8_t HiPNUC_Data[2][82] = {0};
 
HiPNUC_Info_Typedef HiPNUC_Info;


void HiPNUC_Info_Update(HiPNUC_Info_Typedef *HiPNUC_Info,volatile const uint8_t *Data){
 
 
    //HiPNUC_Info->Temperature =  Data[9];
 
    // HiPNUC_Info->Gyro[0] = bit32TOfloat32(( (Data[33] << 24 ) | (Data[32] << 16 ) | (Data[31] << 8 ) | (Data[30]) ));
    // HiPNUC_Info->Gyro[1] = bit32TOfloat32(( (Data[37] << 24 ) | (Data[36] << 16 ) | (Data[35] << 8 ) | (Data[34]) ));
    // HiPNUC_Info->Gyro[2] = bit32TOfloat32(( (Data[41] << 24 ) | (Data[40] << 16 ) | (Data[39] << 8 ) | (Data[38]) ));

    // HiPNUC_Info->Roll  = bit32TOfloat32(( (Data[57] << 24 ) | (Data[56] << 16 ) | (Data[54] << 8 ) | (Data[53]) ));
		// HiPNUC_Info->Pitch = bit32TOfloat32(( (Data[61] << 24 ) | (Data[60] << 16 ) | (Data[59] << 8 ) | (Data[58]) ));
   	// HiPNUC_Info->Yaw	 = bit32TOfloat32(( (Data[65] << 24 ) | (Data[64] << 16 ) | (Data[63] << 8 ) | (Data[62]) ));

 
 }
