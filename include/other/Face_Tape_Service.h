/*******************************************************************************
 * FLOOR TAPE SERVICE                                                          *
 ******************************************************************************/

#ifndef FACE_TAPE_SERVICE_H
#define FACE_TAPE_SERVICE_H
#include "ES_Configure.h"

#include "xc.h"
#include "BOARD.h"
#include "AD.h"
#include "ES_Framework.h"
#include "IO_Ports.h"
#include <stdio.h>

#define FACE_SENSOR AD_PORTV8

#define AD_MAX 1023
#define EMITTER_ON 1
#define EMITTER_OFF 0
#define FACE_TAPE_TICKS 5 // ES Framework timer
#define TEST_DELAY 750000 // Blocking Code For Testing

#define OFF_FACE_TAPE_THRESH 900 // and Less
#define ON_FACE_TAPE_THRESH 990// and Greater

#define NO_TAPE 0

#define FACE_SENSOR_DIR TRISDbits.TRISD3 // Y 4
#define FACE_SENSOR_EN LATDbits.LATD3

struct face_tape_sensor{
    uint16_t on_val;
    uint16_t off_val;
    uint16_t delta;
    int state;
};

uint8_t InitFaceTapeService(uint8_t Priority);

uint8_t PostFaceTapeService(ES_Event ThisEvent);

ES_Event RunFaceTapeService(ES_Event ThisEvent);

void Face_Init();

void Face_Read(uint8_t emitting);

#endif