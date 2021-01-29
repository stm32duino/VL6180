/*******************************************************************************
Copyright © 2019, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED. 
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/



/* vl6180_platform.h STM32 Nucelo F401 single device  using generic cci-i2c
 * trace via swo port some GnuArm eclipse toolset */

#ifndef VL6180_PLATFORM
#define VL6180_PLATFORM

#include "vl6180_def.h"
#include "Wire.h"


struct MyVL6180Dev_t {
  struct VL6180DevData_t Data;
  uint8_t I2cAddr;
  TwoWire *I2cHandle;
};
typedef struct MyVL6180Dev_t *VL6180Dev_t;

#define VL6180DevDataGet(dev, field) (dev->Data.field)
#define VL6180DevDataSet(dev, field, data) (dev->Data.field)=(data)


#endif  /* VL6180_PLATFORM */



