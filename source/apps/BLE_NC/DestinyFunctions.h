#include <stdlib.h>
#include "hal_types.h"

#ifndef _DESTINY_FUNCTIONS_H
#define _DESTINY_FUNCTIONS_H

void DestinySetup();
void DestinyRun();
void GetResults(int matrix1Lines, int matrix1Columns, int matrix2Lines, int matrix2Columns);
void ZeroLine(uint8* line, int size);
void InvertMatrix(double *A, double *A_inv);

#endif