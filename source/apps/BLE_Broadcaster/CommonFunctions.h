#include <stdlib.h>
#include "hal_types.h"

#ifndef _COMMON_FUNCTIONS_H
#define _COMMON_FUNCTIONS_H

void CopyMessage(uint8* destiny, uint8* source);
void GetMessagePayload(uint8* outputAddress, uint8* outputData);
void Transmit(uint8 messageType, uint8* message);

#endif