/*******************************************************************************
 * FLOOR TAPE SERVICE                                                          *
 ******************************************************************************/

#ifndef FLOOR_TAPE_SERVICE_H
#define FLOOR_TAPE_SERVICE_H

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "xc.h"
#include "BOARD.h"
#include "AD.h"
#include "pinout_definitions.h"
#include "stdio.h"

// Floor Tape Sensor Masks
#define FL_MASK 0b0001
#define FR_MASK 0b0010
#define BL_MASK 0b0100
#define BR_MASK 0b1000
#define F_MASK (FL_MASK | FR_MASK)
#define B_MASK (BL_MASK | BR_MASK)
#define NO_TAPE 0

struct tape_sensor{
    uint16_t on_val;
    uint16_t off_val;
    uint16_t delta;
    int state;
    uint8_t mask; 
};

uint8_t InitFloorTapeService(uint8_t Priority);

uint8_t PostFloorTapeService(ES_Event ThisEvent);

ES_Event RunFloorTapeService(ES_Event ThisEvent);

void Tape_Init();

void Tape_Read(uint8_t emitting);

uint8_t Tape_Evaluate(void);

#endif