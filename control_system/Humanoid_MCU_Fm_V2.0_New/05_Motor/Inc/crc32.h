#ifndef __CRC32_H
#define __CRC32_H

#include <stm32f4xx.h>	 
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"

uint32_t crc32_core(uint32_t* ptr, uint32_t len);

#endif  // CRC32_H
