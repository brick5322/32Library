#include "stm32f10x_CRC.h"

CRC_Cls::CRC_Cls()
{
	  int &AHB_CRCON_Bit = *(int *)(0x42420298);
		AHB_CRCON_Bit = 1;
    this->IDR = (uint8_t)0x00;
}

CRC_Cls::~CRC_Cls()
{
    this->CR = true;
    this->IDR = (uint8_t)0x00;
	  int &AHB_CRCON_Bit = *(int *)(0x42420298);
		AHB_CRCON_Bit = 0;
}

uint8_t& CRC_Cls::getTempByte()
{
    return *(uint8_t*)&this->IDR;
}

void CRC_Cls::Reset()
{
    this->CR = true;
}

uint32_t CRC_Cls::CalcCRC(uint32_t Data)
{
    this->DR = Data;
    return this->DR;
}

uint32_t CRC_Cls::CalcCRC(uint32_t* Buffer,uint32_t len)
{
    for(uint32_t i=0;i<len;i++)
        this->DR = Buffer[i];
    return this->DR;
}

uint32_t CRC_Cls::getCurrentRseult()
{
    return this->DR;
}
