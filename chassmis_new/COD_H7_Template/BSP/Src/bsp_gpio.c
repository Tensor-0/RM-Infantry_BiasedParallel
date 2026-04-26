#include "bsp_gpio.h"
#include "main.h"

void BSP_GPIO_Init(void){

//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
//  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);

}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port,ACCEL_CS_Pin,GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port,ACCEL_CS_Pin,GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,GPIO_PIN_SET);
}