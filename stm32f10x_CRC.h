#pragma once
#include "stm32f10x.h"

class CRC_Cls : public Perieh_Cls
{
private:
    volatile uint32_t DR;
    uint8_t IDR_Reserved;
    uint8_t IDR;
    volatile bool CR;

public:
CRC_Cls();
uint8_t& getTempByte();
void Reset();
uint32_t CalcCRC(uint32_t Data);
uint32_t CalcCRC(uint32_t* Buffer,uint32_t len);
uint32_t getCurrentRseult();
~CRC_Cls();
};
