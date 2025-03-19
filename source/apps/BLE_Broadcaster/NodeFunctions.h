#include <stdlib.h>
#include "hal_types.h"

#ifndef _NODE_FUNCTIONS_H
#define _NODE_FUNCTIONS_H

void NodeSetup(uint32 myNumber);
void NodeRun(uint8 phase, uint32 myNumber);
void NodeClean();

#endif