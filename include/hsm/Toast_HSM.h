
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "find_tape_sm.h"
#include "get_parallel_subhsm.h"
#include "pinout_definitions_worked.h"
#include "traverse_sm.h"


#ifndef TOAST_HSM_H
#define	TOAST_HSM_H

uint8_t InitToastHSM(uint8_t priority);

uint8_t PostToastHSM(ES_Event Event);

ES_Event RunToastHSM(ES_Event Event);

#endif	/* TOAST_HSM_H */

