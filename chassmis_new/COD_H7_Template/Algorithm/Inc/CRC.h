#ifndef CRC_H
#define CRC_H

#include "stdint.h"
#include "stdbool.h"

extern  bool Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

extern  bool Verify_CRC8_Check_Sum(unsigned char *pch_message, unsigned int dw_length);

#endif