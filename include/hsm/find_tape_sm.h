#ifndef FIND_TAPE_SM_H
#define FIND_TAPE_SM_H

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "xc.h"
#include "BOARD.h"
#include "AD.h"
#include "pinout_definitions_worked.h"
#include "stdio.h"

uint8_t InitTapeFindSM(void); // Inits Find Sub SM

ES_Event RunFindTapeSM(ES_Event Event); // Runs Find Sub SM

#endif