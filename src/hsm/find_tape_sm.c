#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "find_tape_sm.h"
#include "motors.h"
#include "croissant_serial.h"
#include "RC_Servo.h"
#include "pinout_definitions.h"

// Timeout Values
#define TRANSLATE_TIMEOUT 90
#define TAPE_TIMEOUT 700

#define POWER 45
#define mid_off_kickback 15
#define retract_time 500

// Motor Speed Values
#define TRANSLATE_SPEED 45

// Translate Directions 
#define TRANS_BACKWARD 0
#define TRANS_FORWARD  1

typedef enum {
    InitTFSubState,
    TFSitAndWait,
    Translate,
    STOPANDSAMPLE,
    DEPOSITBALL,
} TFSubState_t;

static const char *StateNames[] = {
    "InitTFSubState",
    "TFSitAndWait",
    "Translate",
    "STOPANDSAMPLE",
    "DEPOSITBALL",
};

static struct lidar_sensors distances;
static uint8_t transDir = TRANS_BACKWARD;
static TFSubState_t CurrentState = InitTFSubState;
static uint8_t prio;
static uint8_t mid_face_counter_lock = 0;

uint8_t InitTapeFindSM(void) {
    ES_Event returnEvent;
    CurrentState = InitTFSubState;
    returnEvent = RunFindTapeSM(INIT_EVENT);
    // Initial Transition Event
    if (returnEvent.EventType == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

ES_Event RunFindTapeSM(ES_Event Event) {
    uint8_t make_transition = FALSE;
    TFSubState_t next_state;

    switch (CurrentState) {
        case InitTFSubState:
            switch (Event.EventType) {
                case ES_INIT:
                    printf("GETTING OFF MID\r\n");
                    next_state = Translate;
                    make_transition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;
                default:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

            //        case TFSitAndWait:
            //            printf("HIT TF HALT\r\n");
            //            switch (Event.EventType) {
            //                case ES_ENTRY:
            //                    printf("TF Sit and Wait\r\n");
            ////                    Toast_StopMtr();
            //                    Event.EventType = ES_NO_EVENT;
            //                    break;
            //
            //                case ES_TIMEOUT:
            //                    if (Event.EventParam == TF_SIT_AND_WAIT_TIMER) {
            //                        next_state = Translate;
            //                        make_transition = TRUE;
            //                        Event.EventType = ES_NO_EVENT;
            //                    }
            //                    break;
            //            }
            //            break;

        case Translate:
            printf("FT Translate\r\n");
            switch (Event.EventType) {
                case ES_ENTRY:
                    // Initialize Watchdog timer
                    ES_Timer_InitTimer(FIND_TAPE_TIMER, TRANSLATE_TIMEOUT);
                    // Set Translate Speed
                    if (transDir == TRANS_FORWARD) {
                        //                        Toast_BothMtrSpeed(TRANSLATE_SPEED);
                        Toast_RightMtrSpeed(TRANSLATE_SPEED + 5);
                        Toast_LeftMtrSpeed(TRANSLATE_SPEED);
                    } else { // transDir == TRANS_BACKWARD
                        //                        Toast_BothMtrSpeed(-TRANSLATE_SPEED);
                        Toast_RightMtrSpeed(-(TRANSLATE_SPEED));
                        Toast_LeftMtrSpeed(-TRANSLATE_SPEED);
                    }

                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless Event Catch
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                case OFF_FACE_TAPE:
                    printf("FT Translate OFF TAPE\r\n");
                    Toast_StopMtr();
                    ES_Timer_StopTimer(FIND_TAPE_TIMER); // Stop Face Tape Watchdog
                    // Reverse Translate Direction
                    printf("OFF FACE TAPE (Translate) with lock counter = %d\r\n", mid_face_counter_lock);
                    if (transDir == TRANS_FORWARD && mid_face_counter_lock >= 6) {
                        Toast_StopMtr();
                        ES_Timer_StopTimer(FIND_TAPE_TIMER); // Stop Face Tape Watchdog
                        next_state = DEPOSITBALL;
                        make_transition = TRUE;

                        // Post Tape Locked Event to Top
                        printf("Face Tape was locked!\r\n");
                        //                    Event.EventType = FACE_TAPE_LOCKED; 
                        mid_face_counter_lock = 0; // Tape lock reset
                        transDir = TRANS_BACKWARD;
                        Event.EventType = ES_NO_EVENT;
                    } else { // transDir == TRANS_BACKWARD
                        transDir == TRANS_FORWARD;
                        next_state = Translate;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_TIMEOUT:
                    printf("FT Translate TO -> SS\r\n");
                    switch (Event.EventParam) {
                        case FIND_TAPE_TIMER:
                            next_state = STOPANDSAMPLE; // Go back to stop and sample
                            make_transition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                            break;
                    }
                    break;

                case ES_EXIT:
                    Toast_StopMtr();
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case DEPOSITBALL:
            switch (Event.EventType) {
                case ES_ENTRY:
                    printf("DEPOSITING BALL\r\n");
                    // extend ramp 
                    RC_SetPulseTime(RAMP_SERVO_PIN, 1000);
                    PWM_SetDutyCycle(BALL_PWM, 50);
                    Event.EventType = ES_NO_EVENT;
                    break;

                case HALL_DETECTED:
                    printf("HALL DETECTED");
                    PWM_SetDutyCycle(BALL_PWM, 0);
                    ES_Timer_InitTimer(MANEUVER_TIMER, retract_time);
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        // reatract ramp
                        RC_SetPulseTime(RAMP_SERVO_PIN, 1950);
                        // transition to find
                        printf("BALL DEPOSITED GOING TO HALT\r\n");
                        next_state = Translate;
                        make_transition = TRUE;
                    }
                    Event.EventType = BALL_DEPOSITED;
                    break;
            }
            break;

        case STOPANDSAMPLE:
            printf("FT Stop and Sampler\n");
            switch (Event.EventType) {
                case ES_ENTRY:
                    // Enable UART
                    UART_EN = 1;
                    //Toast_BothMtrSpeed(-POWER);

                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless Event Catch
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                case DISTANCE_MEAS:;
                    printf("FIND TAPE SS measurement taken\r\n");
                    // Store distance
                    distances = getDistancePacket();
                    // turn off uart enable
                    UART_EN = 0;

                    if (distances.mid >= MAX_DIST) { // Mid went off face, reverse translate directions
                        if (transDir == TRANS_FORWARD) {
                            printf("Translating backwards now\r\n");
                            transDir = TRANS_BACKWARD;
                        } else { // transDir == TRANS_BACKWARD
                            printf("Translating forwards now\r\n");
                            transDir = TRANS_FORWARD;
                        }
                        printf("Reseting Tape Lock\r\n");
                        mid_face_counter_lock = 0; // Tape lock reset
                    } else {
                        mid_face_counter_lock++;
                        printf("Tape Lock Value Now: %d\r\n", mid_face_counter_lock);
                    }

                    next_state = Translate;
                    make_transition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;
                case OFF_FACE_TAPE:
                    printf("FT Translate OFF TAPE\r\n");
                    Toast_StopMtr();
                    ES_Timer_StopTimer(FIND_TAPE_TIMER); // Stop Face Tape Watchdog
                    // Reverse Translate Direction
                    printf("OFF FACE TAPE (Translate) with lock counter = %d\r\n", mid_face_counter_lock);
                    if (transDir == TRANS_FORWARD && mid_face_counter_lock >= 6) {
                        Toast_StopMtr();
                        ES_Timer_StopTimer(FIND_TAPE_TIMER); // Stop Face Tape Watchdog
                        next_state = DEPOSITBALL;
                        make_transition = TRUE;

                        // Post Tape Locked Event to Top
                        printf("Face Tape was locked!\r\n");
                        mid_face_counter_lock = 0; // Tape lock reset
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;
                case ES_TIMEOUT:
                    switch (Event.EventParam) {
                        case MANEUVER_TIMER:
                            Toast_StopMtr();
                            next_state = Translate;
                            make_transition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                            break;
                    }
                    break;

                case ES_EXIT:
                    Toast_StopMtr();
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;
    }

    if (make_transition == TRUE) {
        RunFindTapeSM(EXIT_EVENT);
        CurrentState = next_state;
        RunFindTapeSM(ENTRY_EVENT);
    }

    ES_Tail();
    return (Event);

}