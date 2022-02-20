#ifndef COMPONENTS_H_
#define COMPONENTS_H_

#include "main.h"



void    HTS221_T_Init(uint16_t DeviceAddr);
float   HTS221_T_ReadTemp(uint16_t DeviceAddr);

void    HTS221_H_Init(uint16_t DeviceAddr);
float   HTS221_H_ReadHumidity(uint16_t DeviceAddr);


#endif