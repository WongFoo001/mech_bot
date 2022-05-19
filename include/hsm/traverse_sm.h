#ifndef TRAVERSE_SM_H
#define	TRAVERSE_SM_H

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "motors.h"
#include "pinout_definitions_worked.h"
#include "croissant_serial.h"
#include "lock_track_subhsm.h"

uint8_t InitTTSubHSM(void); 

ES_Event RunTTSubHSM(ES_Event Event); 


#endif	

