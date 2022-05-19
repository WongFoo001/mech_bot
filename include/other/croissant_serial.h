#ifndef CROISSANT_SERIAL_H
#define CROISSANT_SERIAL_H

#include "ES_Configure.h" 
#include "ES_Framework.h"
#include "xc.h"
#include <stdio.h>
#include <BOARD.h>
#include <sys/attribs.h> //needed to use an interrupt
#include <stdint.h>
#include "pinout_definitions.h"
#include "solve_sm.h" // For access to solve state variable

#define DIST_MULTIPLIER 3
#define MAX_DIST 200 //mm
#define NO_MEASURE 0xFF // Default error value
#define OPT_DIST 40
#define LOWER_OPT_THRESH 30
#define UPPER_OPT_THRESH 50
#define LIDAR_NO_SIG 255
#define ROCK_DIST 15

static struct lidar_sensors{
    uint8_t fore;
    uint8_t mid;
    uint8_t aft;
} lidars;

struct lidar_sensors getDistancePacket(void);

char CROISSANT_Init(void);

void PutCroissant(char ch);

char GetCroissant(void);

char IsTransmitEmptyC(void);

ES_Event EatCroissants(void);

char IsReceiveFullC(void);

char IsReceiveEmptyC(void);

void shifter(char place, uint8_t value);

#endif // croissant_serial_h
