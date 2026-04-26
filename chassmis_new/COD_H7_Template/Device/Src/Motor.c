#include "Motor.h"

DM_Motor_Info_Typedef DM_8009_Motor[4] = { //6220电机结构体
 
 [0] = {
   .lost =1,
	 .CAN_Identifier_Info = {
		.Tx_Identifier = 0x01, 
	  .Rx_Identifier = 0x11, 
	 },
	 .Type = DM_8009,
 },
 [1] = {
   .lost =1,
   .CAN_Identifier_Info = {
	  .Tx_Identifier = 0x02, 
	  .Rx_Identifier = 0x12, 
	 },
	 .Type = DM_8009,
 },
 [2] = {
   .lost =1,
   .CAN_Identifier_Info = {
	  .Tx_Identifier = 0x03, 
	  .Rx_Identifier = 0x13, 
	 },
	 .Type = DM_8009,
 },
 [3] = {
   .lost =1,
   .CAN_Identifier_Info = {
	  .Tx_Identifier = 0x04, 
	  .Rx_Identifier = 0x14, 
	 },
	 .Type = DM_8009,
 },

};
DM_Motor_Info_Typedef DM_Yaw_Motor = { //6220电机结构体
   
	 .lost =0,
	 .CAN_Identifier_Info = {
		.Tx_Identifier = 0x01, 
	  .Rx_Identifier = 0x11, 
	 },
	 .Type = DM_6220,
 
};

GIM_Motor_Info_Typedef GIM_8108_Motor[2] = {

  [0]={
		.lost =1,
  	.CAN_Identifier_Info = {
		.Tx_Identifier = 0x01, 
	  .Rx_Identifier = 0x11, 
	 },
  },
	
	 [1]={
		.lost =1,
  	.CAN_Identifier_Info = {
		.Tx_Identifier = 0x02, 
	  .Rx_Identifier = 0x12, 
	 },
  },
	
};



MIT_Motor_Control_Typedef DM_Motor_Control[2];

static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float X_float, float X_min, float X_max, int bits){
    float span = X_max - X_min;
    float offset = X_min;
    return (int) ((X_float-offset)*((float)((1<<bits)-1))/span);
}

static float F_Loop_Constrain(float Input, float minValue, float maxValue);

void MIT_Motor_Command(FDCAN_TxFrame_TypeDef *TxFrame,uint16_t Tx_Identifier,uint8_t CMD){

	 TxFrame->Header.Identifier = Tx_Identifier;
  	
	 TxFrame->Data[0] = 0xFF;
   TxFrame->Data[1] = 0xFF;
 	 TxFrame->Data[2] = 0xFF;
	 TxFrame->Data[3] = 0xFF;
	 TxFrame->Data[4] = 0xFF;
	 TxFrame->Data[5] = 0xFF;
	 TxFrame->Data[6] = 0xFF;
	
	 switch(CMD){
		 
		  case Motor_Enable :
	        TxFrame->Data[7] = 0xFC; 
	    break;
      
			case Motor_Disable :
	        TxFrame->Data[7] = 0xFD; 
      break;
      
			case Motor_Save_Zero_Position :
	        TxFrame->Data[7] = 0xFE; 
			break;
			
			default:
	    break;   
	}
	
  HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);

}

void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque){

   uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
   
	 float P_MAX,V_MAX,T_MAX;
	
   switch(DM_Motor->Type){
		 
		  case DM_8009 :
	       P_MAX = 3.141593f; V_MAX =45.f; T_MAX = 54.f;
	    break;
		 
		  case DM_6220 :
	       P_MAX = 3.141593f; V_MAX = 30.f; T_MAX = 10.f;
	    break;
      
			case DM_4310 :
	       P_MAX = 12.5f; V_MAX = 30.f; T_MAX = 10.f;
			break;
			
			default:
	    break;   
	}
	 
   Postion_Tmp  =  float_to_uint(Postion,-P_MAX,P_MAX,16) ;
   Velocity_Tmp =  float_to_uint(Velocity,-V_MAX,V_MAX,12);
   Torque_Tmp = float_to_uint(Torque,-T_MAX,T_MAX,12);
   KP_Tmp = float_to_uint(KP,0,500,12);
   KD_Tmp = float_to_uint(KD,0,5,12);

   TxFrame->Header.Identifier = DM_Motor->CAN_Identifier_Info.Tx_Identifier;
 	 TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
	 TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
	 TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
	 TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
	 TxFrame->Data[4] = (uint8_t)(KP_Tmp);
	 TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
	 TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
	 TxFrame->Data[7] = (uint8_t)(Torque_Tmp);
 
   HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);

}

void GIM_Motor_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,GIM_Motor_Info_Typedef *GIM_Motor,float Postion, float Velocity, float KP, float KD, float Torque){

   uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
   
   Postion_Tmp  =  float_to_uint(Postion,-3.141593f,-3.141593f,16) ;
   Velocity_Tmp =  float_to_uint(Velocity,-45.f,45.f,12);
   Torque_Tmp = float_to_uint(Torque,-54.f,54.f,12);
   KP_Tmp = float_to_uint(KP,0,500,12);
   KD_Tmp = float_to_uint(KD,0,5,12);

   TxFrame->Header.Identifier = GIM_Motor->CAN_Identifier_Info.Tx_Identifier;
 	 TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
	 TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
	 TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
	 TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
	 TxFrame->Data[4] = (uint8_t)(KP_Tmp);
	 TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
	 TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
	 TxFrame->Data[7] = (uint8_t)(Torque_Tmp);
  
   HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);

}


void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Data,DM_Motor_Info_Typedef *DM_Motor)
{
	
		if(*Identifier != DM_Motor->CAN_Identifier_Info.Rx_Identifier) return;
			
    float P_MAX,V_MAX,T_MAX;
	
   switch(DM_Motor->Type){
      
		  case DM_6220 :
	       P_MAX = 3.141593f; V_MAX = 30.f; T_MAX = 10.f;
	    break;
      
			case DM_4310 :
	       P_MAX = 12.5f; V_MAX = 30.f; T_MAX = 10.f;
			break;
				case DM_8009 :
	       P_MAX = 3.141593f; V_MAX = 45.f; T_MAX = 54.f;
			break;
			
			default:
	    break;   

   }
	 
	 
    DM_Motor->Data.State = Data[0]>>4;
	 	DM_Motor->Oline_cnt = 250;
		DM_Motor->Data.P_int = ((uint16_t)(Data[1]) <<8) | ((uint16_t)(Data[2]));
		DM_Motor->Data.V_int = ((uint16_t)(Data[3]) <<4) | ((uint16_t)(Data[4])>>4);
		DM_Motor->Data.T_int = ((uint16_t)(Data[4]&0xF) <<8) | ((uint16_t)(Data[5]));
		DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-T_MAX,T_MAX,12);
		DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-P_MAX,P_MAX,16);
    DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-V_MAX,V_MAX,12);
	 	 
    DM_Motor->Data.Temperature_MOS   = (float)(Data[6]);
		DM_Motor->Data.Temperature_Rotor = (float)(Data[7]);


}
	 
void GIM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Data,GIM_Motor_Info_Typedef *GIM_Motor)
{
	
		if(*Identifier != GIM_Motor->CAN_Identifier_Info.Rx_Identifier ) return;
	   GIM_Motor->Data.State = Data[0]>>4;
			GIM_Motor->Oline_cnt = 250;
		 GIM_Motor->Data.P_int = ((uint16_t)(Data[1]) <<8) | ((uint16_t)(Data[2]));
		 GIM_Motor->Data.V_int = ((uint16_t)(Data[3]) <<4) | ((uint16_t)(Data[4])>>4);
		 GIM_Motor->Data.T_int = ((uint16_t)(Data[4]&0xF) <<8) | ((uint16_t)(Data[5]));
     GIM_Motor->Data.Torque=  uint_to_float( GIM_Motor->Data.T_int,-54.f,54.f,12);
		 GIM_Motor->Data.Position=uint_to_float( GIM_Motor->Data.P_int,-95.5f,95.5f,16);
     GIM_Motor->Data.Velocity=uint_to_float( GIM_Motor->Data.V_int,-45.0f,45.0f,12);
		


} 





static float F_Loop_Constrain(float Input, float minValue, float maxValue)
{
  if (maxValue < minValue)
  {
    return Input;
  }
  
  float len = maxValue - minValue;    

  if (Input > maxValue)
  {
      do{
          Input -= len;
      }while (Input > maxValue);
  }
  else if (Input < minValue)
  {
      do{
          Input += len;
      }while (Input < minValue);
  }
  return Input;
}
//------------------------------------------------------------------------------


//static float Encoder_To_Angle(DM_Motor_Multi_GeneralInfo_Typedef *Info,float Torque_Ratio,uint16_t MAXencoder)
//{	
//  float Encoder_Err = 0.f;
//  
//  /* check the motor init */
//  if(Info->Initlized != true)
//  {
//    /* update the last encoder */
//    Info->Last_Encoder = Info->Encoder;

//    /* reset the angle */
//    Info->Angle = Info->Encoder/(MAXencoder*Torque_Ratio)*360.f;

//    /* config the init flag */
//    Info->Initlized = true;
//  }
//  
//  Encoder_Err = Info->Encoder - Info->Last_Encoder;
//  
//  /* 0 -> MAXencoder */		
//  if(Encoder_Err > MAXencoder*0.5f)
//  {
//    Info->Angle += (float)(Encoder_Err - MAXencoder)/(MAXencoder*Torque_Ratio)*360.f;
//  }
//  /* MAXencoder-> 0 */		
//  else if(Encoder_Err < -MAXencoder*0.5f)
//  {
//    Info->Angle += (float)(Encoder_Err + MAXencoder)/(MAXencoder*Torque_Ratio)*360.f;
//  }
//  else
//  {
//    Info->Angle += (float)(Encoder_Err)/(MAXencoder*Torque_Ratio)*360.f;
//  }
//  
//  /* update the last encoder */
//  Info->Last_Encoder = Info->Encoder;
//  
//  /* loop constrain */
//  Info->Angle = F_Loop_Constrain(Info->Angle,-180.f,180.f);

//  return Info->Angle;
//}


