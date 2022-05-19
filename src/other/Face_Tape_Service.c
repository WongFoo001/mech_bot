#include "Face_Tape_Service.h"

// Define when face tape sensor is in air
#define IN_AIR_READING 10

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

//#define TAPE_VERBOSE  // TEST PRINTsOUTS
static uint8_t P;

static struct face_tape_sensor FS; // Face Sensor

//static uint16_t face_sensor;
//static uint16_t on_reading;
//static uint16_t off_reading;

void Face_Init() {
    // Configure enable IO pin
    FACE_SENSOR_DIR = 0; // Set enable direction to "output"
    FACE_SENSOR_EN = EMITTER_OFF; // Set to off by default (power efficient)
    struct face_tape_sensor init = {.on_val = 0, .off_val = 0, .delta = 0, .state = 0};

    FS = init;
}

void Face_Read(uint8_t emitting) {
    //    printf("reading tape \r\n");
    if (emitting == EMITTER_ON) {
        FS.on_val = AD_ReadADPin(FACE_SENSOR);
#ifdef TAPE_VERBOSE
        printf("ON   read tape looks like: %3d\r\n", FS.on_val);
#endif
    } else { // EMITTING_OFF
        FS.off_val = AD_MAX - AD_ReadADPin(FACE_SENSOR);

#ifdef TAPE_VERBOSE
//        printf("OFF   read tape looks like: %3d\r\n", FS.off_val);
#endif
    }

}

uint8_t InitFaceTapeService(uint8_t Priority) {
    ES_Event ThisEvent;
    Face_Init();
    P = Priority;
    ThisEvent.EventType = ES_INIT;
    ES_Timer_InitTimer(FACE_TAPE_SERVICE_TIMER, FACE_TAPE_TICKS);
    if (ES_PostToService(P, ThisEvent) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

uint8_t PostFaceTapeService(ES_Event ThisEvent) {
    return ES_PostToService(P, ThisEvent);
}

ES_Event RunFaceTapeService(ES_Event ThisEvent) {
    static ES_EventTyp_t prevFace = OFF_FACE_TAPE;
    //static char last_tape = NO_TAPE;
    ES_Event faceEvent;
//    faceEvent.EventType = ES_NO_EVENT; // Assume no event by default
    ES_EventTyp_t curFace = prevFace;
    switch (ThisEvent.EventType) {
        case ES_INIT:
            break;

        case ES_TIMERACTIVE:
        case ES_TIMERSTOPPED:
            break;

        case ES_TIMEOUT:
            ES_Timer_InitTimer(FACE_TAPE_SERVICE_TIMER, FACE_TAPE_TICKS);
            if (AD_IsNewDataReady()) {
//                printf("data ready \r\n");
                if (FACE_SENSOR_EN == EMITTER_ON) {
                    Face_Read(EMITTER_ON);
                    FACE_SENSOR_EN = EMITTER_OFF;
//                    printf("read with emitter on \r\n");
                } else {
                    Face_Read(EMITTER_OFF);
                    FACE_SENSOR_EN = EMITTER_ON;
//                    printf("read with emitter off \r\n");
                    FS.delta = FS.on_val - FS.off_val;
//                    printf("Face Reading: %d\r\n", FS.delta);
                    if (FS.delta == 1023){
                        curFace = OFF_FACE_TAPE;
                        prevFace = curFace;
                    }
                    else{
                        if(FS.delta > ON_FACE_TAPE_THRESH){
                            curFace = ON_FACE_TAPE;
//                            printf("FACE TAPE SEEN!");
//                            printf(" | Reading: %d\r\n", FS.delta);
                            if(prevFace != curFace){
                                faceEvent.EventType = curFace;
    //                            printf("done event type \r\n");
                                faceEvent.EventParam = (uint16_t)FS.delta;
    //                            printf("done delta insert \r\n");
                                prevFace = curFace;
    //                            printf("done setting previous face\r\n");
                                PostToastHSM(faceEvent);
    //                            printf("done posting\r\n");
                            }


                        }else if(FS.delta < OFF_FACE_TAPE_THRESH){
                            curFace = OFF_FACE_TAPE;
//                            printf("FACE TAPE NOT SEEN!");
//                            printf(" | Reading: %d\r\n", FS.delta);
                            if(prevFace != curFace){
                                faceEvent.EventType = curFace;
    //                            printf("done event type \r\n");
                                faceEvent.EventParam = (uint16_t)FS.delta;
    //                            printf("done delta insert \r\n");
                                prevFace = curFace;
    //                            printf("done setting previous face\r\n");
                                PostToastHSM(faceEvent);
    //                            printf("done posting\r\n");
                            }
                        }
                    }
                }
            }
            break;
    }
//    return (faceEvent);
}
