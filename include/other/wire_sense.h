#ifndef WIRE_SENSE_H
#define WIRE_SENSE_H

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT
#include "stdio.h"
#include "BOARD.h"
#include "AD.h"
#include "ES_Framework.h"

#define WIRE 120
#define NO_WIRE 80

#define FORE_TW_MASK 0x01
#define AFT_TW_MASK  0x02

uint16_t* getTrackWire(void);

uint8_t InitWireService(uint8_t Priority);

uint8_t PostWireService(ES_Event E);

ES_Event RunWireService(ES_Event E);

#endif