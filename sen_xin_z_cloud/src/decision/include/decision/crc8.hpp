#pragma once

#include <cstdint>

namespace crc8
{
extern const unsigned char CRC8_INIT;
extern const std::uint16_t CRC_INIT;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

std::uint16_t Get_CRC16_Check_Sum(std::uint8_t *pchMessage, std::uint32_t dwLength, std::uint16_t wCRC);
std::uint32_t Verify_CRC16_Check_Sum(std::uint8_t *pchMessage, std::uint32_t dwLength);
void Append_CRC16_Check_Sum(std::uint8_t *pchMessage, std::uint32_t dwLength);
}
