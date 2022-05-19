#include "ES_Configure.h"
#include "ES_Framework.h"
#include "xc.h"
#include "BOARD.h"
#include "pinout_definitions.h"
#include "AD.h"
#include "stdio.h"
#include "croissant_serial.h"

#ifndef TOASTEVENTCHECKER_H
#define	TOASTEVENTCHECKER_H

uint8_t ToastCheckHall(void);

uint8_t ToastCheckBeacon(void);

uint8_t ToastCheckUART(void);



#endif	/* TOASTEVENTCHECKER_H */

