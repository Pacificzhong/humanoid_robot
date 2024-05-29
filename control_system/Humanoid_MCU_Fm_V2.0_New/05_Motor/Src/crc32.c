
#include "crc32.h"

uint32_t crc32_core(uint32_t *ptr, uint32_t len);

// CRC校验32位计算
inline uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  unsigned int i;
  char bits;
  for (i = 0; i < len; i++)
  {
    xbit = (uint32_t)1 << 31;
    data = ptr[i];
    for (bits = 0; bits < 32; bits++)
    {
      if (CRC32 & 0x80000000)
      {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      }
      else
        CRC32 <<= 1;
      if (data & xbit)
        CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  return CRC32;
}
