#include "Floor_Tape_Service.h"

typedef enum {
    STATIC_OFF = 0,
    STATIC_ON = 1,
    ON_EDGE = 2,
    OFF_EDGE = 3
} Floor_Tape_States;

typedef enum {
    FRONT_R = 0,
    FRONT_L = 1,
    BACK_R = 2,
    BACK_L = 3
} tape_sensor_ref;



#define NUM_FLOOR_SENS 4

#define AD_MAX 1023
#define EMITTER_ON 1
#define EMITTER_OFF 0
#define TAPE_TICKS 10 // ES Framework timer
#define TEST_DELAY 750000 // Blocking Code For Testing

#define ON_BLOCK_TEST
#ifdef ON_BLOCK_TEST
#define OFF_TAPE_THRESH 500 // and Less
#define ON_TAPE_THRESH 750 // and Greater
#else
#define OFF_TAPE_THRESH 125 // and Less
#define ON_TAPE_THRESH 250 // and Greater
#endif

//#define TAPE_VERBOSE  // TEST PRINTsOUTS
static uint8_t MyPriority;
static ES_EventTyp_t prevFloor = ES_NO_EVENT;

static struct tape_sensor AFS[NUM_FLOOR_SENS];// All Floor Sensors

void Tape_Init() {
    // Configure enable IO pin
    TAPE_SENSOR_DIR = 0; // Set enable direction to "output"
    TAPE_SENSOR_EN = EMITTER_ON; // Set to off by default (power efficient)
    struct tape_sensor init = {.on_val = 0, .off_val = 0, .delta = 0, .state = 0};

    // Configure tape sensor struct array
    AFS[FRONT_R] = init;
    AFS[FRONT_L] = init;
    AFS[BACK_R] = init;
    AFS[BACK_L] = init;
    
    AFS[FRONT_R].mask = FR_MASK;
    AFS[FRONT_L].mask = FL_MASK;
    AFS[BACK_R].mask = BR_MASK;
    AFS[BACK_L].mask = BL_MASK;
}

void Tape_Read(uint8_t emitting) {
//    printf("reading tape \r\n");
    if (emitting == EMITTER_ON) {
        AFS[FRONT_R].on_val = AD_ReadADPin(FORE_R);
        AFS[FRONT_L].on_val = AD_ReadADPin(FORE_L);
        AFS[BACK_L].on_val = AD_ReadADPin(AFT_L);
        AFS[BACK_R].on_val = AD_ReadADPin(AFT_R);
#ifdef TAPE_VERBOSE
        printf("ON   read tape looks like: br(%3d) bl(%3d) fr(%3d) fl(%3d) \r\n", AFS[FRONT_L].on_val,
                                                                 AFS[FRONT_R].on_val,
                                                                 AFS[BACK_L].on_val,
                                                                 AFS[BACK_R].on_val);
#endif
    } else { // EMITTING_OFF
        AFS[FRONT_L].off_val = AD_MAX - AD_ReadADPin(FORE_L);
        AFS[FRONT_R].off_val = AD_MAX - AD_ReadADPin(FORE_R);
        AFS[BACK_L].off_val = AD_MAX - AD_ReadADPin(AFT_L);
        AFS[BACK_R].off_val = AD_MAX - AD_ReadADPin(AFT_R);
#ifdef TAPE_VERBOSE
        printf("OFF  read tape looks like: br(%3d) bl(%3d) fr(%3d) fl(%3d) \r\n", AFS[FRONT_L].off_val,
                                                                     AFS[FRONT_R].off_val,
                                                                     AFS[BACK_L].off_val,
                                                                     AFS[BACK_R].off_val);
#endif
    }
    
}

uint8_t Tape_Evaluate(void) {
    
//    printf("evaluating tape \r\n");
    // Compute difference between off and on values
    AFS[FRONT_L].delta = AFS[FRONT_L].on_val - AFS[FRONT_L].off_val;
    AFS[FRONT_R].delta = AFS[FRONT_R].on_val - AFS[FRONT_R].off_val;
    AFS[BACK_L].delta = AFS[BACK_L].on_val - AFS[BACK_L].off_val;
    AFS[BACK_R].delta = AFS[BACK_R].on_val - AFS[BACK_R].off_val;
#ifdef TAPE_VERBOSE
    printf("tape eval looks like      :fl(%3d) fr(%3d) bl(%d) br(%3d) \r\n", AFS[FRONT_L].delta, 
                                                               AFS[FRONT_R].delta, 
                                                               AFS[BACK_L].delta, 
                                                               AFS[BACK_R].delta);
#endif
    // Hysteresis and Event Evaluation
    char ret = NO_TAPE;
    
    int i; // Index variable
    for (i = 0; i < NUM_FLOOR_SENS; i++) {
        switch(AFS[i].state) {
            case(STATIC_OFF):
                // Check for crossing of the HIGH threshold
                if(AFS[i].delta > ON_TAPE_THRESH) {
                    // Change State
                    AFS[i].state = ON_EDGE;
                }
                break;
            case(STATIC_ON):
                // Check for crossing of the LOW threshold
                if (AFS[i].delta < OFF_TAPE_THRESH) {
                    // Change State
                    AFS[i].state = OFF_EDGE;
                }
                break;
            case(ON_EDGE):
                // Check for crossing of the LOW threshold
                if (AFS[i].delta < OFF_TAPE_THRESH) {
                    // Change State
                    AFS[i].state = OFF_EDGE;
                } else { // Else move to STATIC ON
                    AFS[i].state = STATIC_ON;
                    ret |= AFS[i].mask;
                }
                break;
            case(OFF_EDGE):
                // Check for crossing of the HIGH threshold
                if (AFS[i].delta > ON_TAPE_THRESH) {
                    // Change State
                    AFS[i].state = ON_EDGE;
                } else { // Else move to STATIC ON
                    AFS[i].state = STATIC_OFF;
                }
                break;
            default:
                // ERROR
                printf("A serious error occurred in floor sensor service...\r\n");
        }
#ifdef TAPE_VERBOSE
        printf("AFS[%d] state: %d | ", i, AFS[i].state);
#endif
    }
#ifdef TAPE_VERBOSE
    printf("\r\n");
#endif
    return ret;
}

uint8_t InitFloorTapeService(uint8_t Priority) {
    ES_Event ThisEvent;
    Tape_Init();
    MyPriority = Priority;
    ThisEvent.EventType = ES_INIT;
    ES_Timer_InitTimer(FLOOR_SERVICE_TIMER, TAPE_TICKS);
    if (ES_PostToService(MyPriority, ThisEvent) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

uint8_t PostFloorTapeService(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event RunFloorTapeService(ES_Event ThisEvent) {
    static ES_EventTyp_t lastEvent = OFF_FLOOR_TAPE;
    static char last_tape = NO_TAPE;
    ES_Event floorEvent;
    floorEvent.EventType = ES_NO_EVENT; // Assume no event by default
    ES_EventTyp_t curFloor;
    //printf("FLOOR TAPE SERVICE CALL\r\n");
    switch (ThisEvent.EventType) {
        case ES_INIT:
            break;

        case ES_TIMERACTIVE:
        case ES_TIMERSTOPPED:
            break;

        case ES_TIMEOUT:
            if (ThisEvent.EventParam == FLOOR_SERVICE_TIMER){
                ES_Timer_InitTimer(FLOOR_SERVICE_TIMER, TAPE_TICKS);
                if (TAPE_SENSOR_EN == EMITTER_ON) {
                    Tape_Read(EMITTER_ON);
                    TAPE_SENSOR_EN = EMITTER_OFF;
                } else {
                    Tape_Read(EMITTER_OFF);
                    TAPE_SENSOR_EN = EMITTER_ON;
                    uint8_t tape_states = Tape_Evaluate();
                    if (tape_states != NO_TAPE && tape_states != last_tape) {
                        floorEvent.EventType = TAPE_DETECTED;
                        floorEvent.EventParam = tape_states;
                        last_tape = tape_states;
                    }
                    if (floorEvent.EventType != ES_NO_EVENT){ // Don't spam no_events
                        PostToastHSM(floorEvent);
                    }
                }
            }
            break;
        default:
            break;
    }
    return (floorEvent);
}
