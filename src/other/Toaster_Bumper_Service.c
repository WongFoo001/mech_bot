#include "Toaster_Bumper_Service.h"

#define BATTERY_DISCONNECT_THRESHOLD 175
#define BUMPER_TICKS 100

#define FRONT_LEFT_BUMPER        PORTEbits.RE3 // Z05
#define FRONT_CENTER_BUMPER      PORTFbits.RF1 // Z04
#define FRONT_RIGHT_BUMPER       PORTEbits.RE4 // Z03

#define REAR_LEFT_BUMPER         PORTEbits.RE1 // Z09
#define REAR_CENTER_BUMPER       PORTDbits.RD8 // Z08
#define REAR_RIGHT_BUMPER        PORTEbits.RE2 // Z07

#define FRONT_LEFT_BUMPER_TRIS   TRISEbits.TRISE3
#define FRONT_CENTER_BUMPER_TRIS TRISFbits.TRISF1
#define FRONT_RIGHT_BUMPER_TRIS  TRISEbits.TRISE4

#define REAR_LEFT_BUMPER_TRIS    TRISEbits.TRISE1
#define REAR_CENTER_BUMPER_TRIS  TRISDbits.TRISD8
#define REAR_RIGHT_BUMPER_TRIS   TRISEbits.TRISE2



static uint8_t MyPriority;
static ES_EventTyp_t prevBump = NOT_BUMPED;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/
void Bumpers_Init() {
    FRONT_LEFT_BUMPER_TRIS = 1;
    FRONT_CENTER_BUMPER_TRIS = 1;
    FRONT_RIGHT_BUMPER_TRIS = 1;
    REAR_LEFT_BUMPER_TRIS = 1;
    REAR_CENTER_BUMPER_TRIS = 1; 
    REAR_RIGHT_BUMPER_TRIS = 1;
}

uint16_t ReadBumpers(void) { // Bumpers are active high
    uint8_t bumpers = FRONT_LEFT_BUMPER + ((FRONT_CENTER_BUMPER) << 1) + ((FRONT_RIGHT_BUMPER) << 2) + ((REAR_LEFT_BUMPER) << 3) + ((REAR_CENTER_BUMPER) << 4) + ((REAR_RIGHT_BUMPER) << 5);
    //printf("Bumpers: %d\r\n", bumpers);
    return (bumpers);
    
}

uint8_t InitBumperService(uint8_t Priority) {
    ES_Event ThisEvent;
    printf("BUMPER SERVICE INITIALIZED\r\n");
    MyPriority = Priority;
    ES_Timer_InitTimer(BUMPER_SERVICE_TIMER, BUMPER_TICKS);

    // post the initial transition event
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

uint8_t PostBumperService(ES_Event ThisEvent) {
    //printf("Bumper Posting\r\n");
    return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event RunBumperService(ES_Event ThisEvent) {
    //printf("BUMPER SERVICE RAN\r\n");
    ES_EventTyp_t curBumper;
    ES_Event bumpEvent;
    bumpEvent.EventType = ES_NO_EVENT; // assume no errors
    
    switch (ThisEvent.EventType) {
        case ES_INIT:
            // No hardware initialization or single time setups, those
            // go in the init function above.
            //
            // This section is used to reset service for some reason
            break;

        case ES_TIMERACTIVE:
        case ES_TIMERSTOPPED:
            break;

        case ES_TIMEOUT:
            //if (ThisEvent.EventParam == BUMPER_SERVICE_TIMER){
                ES_Timer_InitTimer(BUMPER_SERVICE_TIMER, BUMPER_TICKS);
                uint16_t bumpValue = ReadBumpers();
                //printf("Bumper Timed out!\r\n");
                if (bumpValue > 0) {
                    curBumper = BUMP_DANGER;
                } else {
                    curBumper = NOT_BUMPED;
                }
                //printf("BUMP VALUE: %d\r\n", bumpValue);
                if (curBumper != prevBump) { // Only post Bumper Pressed Event                   
                    bumpEvent.EventParam = bumpValue;
                    bumpEvent.EventType = curBumper;
                    prevBump = curBumper; 
                    printf("BUMP EVENT POSTED FROM SERVICE\r\n");
                    printf("WITH PARAM: %d\r\n", bumpValue);
                 //   PostBumperService(bumpEvent);
                    PostToastHSM(bumpEvent);
                }
                //prevBump = curBumper; // Store event history
            //}
            //printf("WON HELP!!!\r\n");
            break;
    }
    return bumpEvent;
}

/*******************************************************************************
 * PRIVATE FUNCTIONs                                                           *
 ******************************************************************************/

