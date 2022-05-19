#ifndef TOASTER_BUMPER_SERVICE_H  // <- This should be changed to your own guard on both
#define TOASTER_BUMPER_SERVICE_H  //    of these lines

#include "ES_Configure.h"
#include "xc.h"
#include "BOARD.h"
#include "AD.h"
#include "ES_Framework.h"
#include "Toaster_Bumper_Service.h"
#include <stdio.h>

#define BUMPER_FL_MASK 0b000001
#define BUMPER_F_MASK  0b000010
#define BUMPER_FR_MASK 0b000100
#define BUMPER_FNL_MASK (BUMPER_FL_MASK | BUMPER_F_MASK)
#define BUMPER_FNR_MASK (BUMPER_FR_MASK | BUMPER_F_MASK)
#define BUMPER_BL_MASK 0b001000
#define BUMPER_B_MASK  0b010000
#define BUMPER_BR_MASK 0b100000
#define BUMPER_BNL_MASK (BUMPER_BL_MASK | BUMPER_B_MASK)
#define BUMPER_BNR_MASK (BUMPER_BR_MASK | BUMPER_B_MASK)
#define BUMPER_L_MASK (BUMPER_FL_MASK | BUMPER_BL_MASK)
#define BUMPER_R_MASK (BUMPER_FR_MASK | BUMPER_BR_MASK)

void Bumpers_Init();

uint16_t ReadBumpers(void);

uint8_t InitBumperService(uint8_t Priority);

uint8_t PostBumperService(ES_Event ThisEvent);

ES_Event RunBumperService(ES_Event ThisEvent);



#endif /* TemplateService_H */

