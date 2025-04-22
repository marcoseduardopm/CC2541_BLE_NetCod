#include <stdlib.h>
#include "hal_types.h"

#ifndef _COMMON_FUNCTIONS_H
#define _COMMON_FUNCTIONS_H

void TurnLED(uint8 led);
void IncludeDevices();
void ClearMessages();
uint8 GetMessagePayload(uint8* outputAddress, uint8* outputData);
void Transmit(uint8 messageType, uint8 mask, uint16* message);
void CodingMatrixConfig();
//void PrintMatrix(uint8* matrix, int lines, int columns);

#endif