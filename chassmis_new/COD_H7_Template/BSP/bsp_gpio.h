#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

extern void BSP_GPIO_Init(void);

extern void BMI088_ACCEL_NS_L(void);

extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);

extern void BMI088_GYRO_NS_H(void);

#endif 