#ifndef GET_PARALLEL_SUBHSM_H
#define	GET_PARALLEL_SUBHSM_H

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "motors.h"
#include "croissant_serial.h"
#include "pinout_definitions_worked.h"


uint8_t InitGPSubHSM(void); 

ES_Event RunGPSubHSM(ES_Event Event); 


#endif	

