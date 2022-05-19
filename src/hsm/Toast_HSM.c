#include "Toast_HSM.h"

// Tape Evade Directions
#define TAPE_BACKWARD 0
#define TAPE_FORWARD  1

// Timeout Values
#define BEACON_TIMEOUT         2000
#define CORRECTIONAL_SCAN_TIME 75
#define RANDOM_WALK_TIMEOUT    3000
#define CONTINUATION_TIMEOUT   600
#define EVADE_TIMEOUT          750
#define EXIT_TIMEOUT           300
#define SIT_AND_WAIT_TIMEOUT   25

// Motor Speed Values
#define BEACON_SCAN_SPEED     35
#define CORRECTION_SCAN_SPEED 50
#define FORWARD_SPEED         50
#define EVADE_SPEED           40

// Pivot Angle Values
#define EVADE_PIVOT_ANGLE 40

// Floor Tape Sensor Masks
#define FL_MASK 0b0001
#define FR_MASK 0b0010
#define BL_MASK 0b0100
#define BR_MASK 0b1000
#define F_MASK (FL_MASK | FR_MASK)
#define B_MASK (BL_MASK | BR_MASK)
#define NO_TAPE 0

typedef enum {
    InitToastHSMState,
    FindBeacon,
    FlyToBeacon,
    EvadeTape,
    GetParallel,
    FindTrack,
    FindTape,
    ExitVat
} ToastState_t;

static const char *StateNames[] = {
    "InitToastHSMState",
    "FindBeacon",
    "FlyToBeacon",
    "EvadeTape",
    "GetParallel",
    "FindTrack",
    "FindTape",
    "ExitVat"
};

static ToastState_t CurrentState = InitToastHSMState;
static uint8_t prio;
static uint8_t BeaconFound = FALSE;
static uint16_t TapeReading = 0;
static uint8_t TapeEvadeDir = TAPE_FORWARD;

uint8_t InitToastHSM(uint8_t priority) {
    prio = priority;
    CurrentState = InitToastHSMState;

    // Post initial transition event
    if (ES_PostToService(prio, INIT_EVENT) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

uint8_t PostToastHSM(ES_Event Event) {
    return ES_PostToService(prio, Event);
}

ES_Event RunToastHSM(ES_Event Event) {
    uint8_t MakeTransition = FALSE;
    ToastState_t NextState;

    ES_Tattle();

    // Top Level State Switch Statement
    switch (CurrentState) {
        case InitToastHSMState:
            switch (Event.EventType) {
                case ES_INIT:
                    // Initialize Substate machines
                    InitGPSubHSM();
                    InitTTSubHSM();
                    InitTapeFindSM();

                    // Initial state transition
                    NextState = FindBeacon;
                    MakeTransition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;
                default:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case FindBeacon:
            switch (Event.EventType) {
                case ES_ENTRY:
                    // ZeroPT CW Turn 
                    Toast_ZeroPt_CW(BEACON_SCAN_SPEED);
                    // Initialize Watchdog timer
                    ES_Timer_InitTimer(FIND_WATCHDOG_TIMER, BEACON_TIMEOUT);
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless Event Catch
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                case SEE_BEACON:
                    ES_Timer_StopTimer(FIND_WATCHDOG_TIMER); // Stop Beacon Watchdog
                    Toast_ZeroPt_CCW(CORRECTION_SCAN_SPEED); // Correctional Adjustment
                    ES_Timer_InitTimer(MANEUVER_TIMER, CORRECTIONAL_SCAN_TIME);
                    BeaconFound = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    switch (Event.EventParam) {
                        case FIND_WATCHDOG_TIMER:
                            //printf("HEY DUDE THE FIND WATCHDOG TIMER ENDED\r\n");
                            BeaconFound = FALSE;
                            NextState = FlyToBeacon;
                            MakeTransition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                            break;
                        case MANEUVER_TIMER:
                            NextState = FlyToBeacon;
                            MakeTransition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                            break;
                    }
                    break;

                case BUMP_DANGER:
                    printf("BUMP DANGER TRANSITION TO GET PARALLEL\r\n");
                    //ES_Timer_InitTimer(FIND_WATCHDOG_TIMER, 7000);
                    TapeReading = Event.EventParam; // Store Tape Reading
                    NextState = GetParallel;
                    MakeTransition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case FlyToBeacon:
            switch (Event.EventType) {
                case ES_ENTRY:
                    if (BeaconFound == FALSE) {
                        // Initialize Watchdog timer
                        ES_Timer_InitTimer(FIND_WATCHDOG_TIMER, RANDOM_WALK_TIMEOUT);
                    }
                    // Set Motors Going Forwards
                    Toast_BothMtrSpeed(FORWARD_SPEED);
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless Event Catch
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                case LOST_BEACON:
                    ES_Timer_StopTimer(FIND_WATCHDOG_TIMER); // Stop Random Walk Timer
                    ES_Timer_InitTimer(FIND_WATCHDOG_TIMER, CONTINUATION_TIMEOUT); // Start Continuation Timer
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    switch (Event.EventParam) {
                        case FIND_WATCHDOG_TIMER:
                            NextState = FindBeacon;
                            MakeTransition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                            break;
                    }
                    break;

                case TAPE_DETECTED:
                    TapeReading = Event.EventParam; // Store Tape Reading
                    NextState = EvadeTape;
                    MakeTransition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;

                case BUMP_DANGER:
                    printf("BUMP DANGER TRANSITION TO GET PARALLEL\r\n");
                    //ES_Timer_InitTimer(FIND_WATCHDOG_TIMER, 7000);
                    TapeReading = Event.EventParam; // Store Tape Reading
                    NextState = GetParallel;
                    MakeTransition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;

                    //                case ES_EXIT:
                    //                    printf("Get Parallel Entry: Starting transition timer\r\n");
                    //                    ES_Timer_InitTimer(GP_SIT_AND_WAIT_TIMER, SIT_AND_WAIT_TIMEOUT);
                    //                    Event.EventType = ES_NO_EVENT;
                    //                    break;
            }
            break;

        case EvadeTape:
            switch (Event.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(FIND_WATCHDOG_TIMER, EVADE_TIMEOUT);
                    if ((TapeReading & FL_MASK) || (TapeReading & FR_MASK)) {
                        Toast_BothMtrSpeed(-EVADE_SPEED);
                        TapeEvadeDir = TAPE_BACKWARD;
                    } else if ((TapeReading & BL_MASK) || (TapeReading & BR_MASK)) {
                        Toast_BothMtrSpeed(EVADE_SPEED);
                        TapeEvadeDir = TAPE_FORWARD;
                    } else {
                        printf("ERROR IN TAPE EvadeTape->ES_ENTRY\r\n");
                        ES_Timer_StopTimer(FIND_WATCHDOG_TIMER);
                    }
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless Event Catch
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    switch (Event.EventParam) {
                        case FIND_WATCHDOG_TIMER:
                            if (TapeReading & FL_MASK) {
                                Toast_Pivot_FCW(EVADE_PIVOT_ANGLE, EVADE_SPEED);
                            } else if (TapeReading & FR_MASK) {
                                Toast_Pivot_FCCW(EVADE_PIVOT_ANGLE, EVADE_SPEED);
                            } else { // Either Reverse Tape Evade goes straight back to find beacon
                                NextState = FindBeacon;
                                MakeTransition = TRUE;
                            }
                            Event.EventType = ES_NO_EVENT;
                            break;

                        case MANEUVER_TIMER:
                            BeaconFound = FALSE;
                            NextState = FindBeacon;
                            MakeTransition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                            break;
                    }
                    break;

                case TAPE_DETECTED:
                    // Transition back to EvadeTape -> Will recall entry function perhaps? Might need test
                    TapeReading = Event.EventParam; // Store Tape Reading
                    NextState = EvadeTape;
                    MakeTransition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case GetParallel:
            Event = RunGPSubHSM(Event); // Pass to find subHSM first
            switch (Event.EventType) {
                case ES_ENTRY:
                    Event.EventType = ES_NO_EVENT;
                    break;
                case SEARCH_TRACKWIRE:
                    NextState = FindTrack;
                    MakeTransition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;
                    //                case ES_EXIT:
                    //                    ES_Timer_InitTimer(TT_SIT_AND_WAIT_TIMER, SIT_AND_WAIT_TIMEOUT);
                    //                    Event.EventType = ES_NO_EVENT;
                    //                    break;
            }
            break;

        case FindTrack:
            Event = RunTTSubHSM(Event);
            switch (Event.EventType) {
                case ES_ENTRY:
                    Event.EventType = ES_NO_EVENT;
                    break;
                case TRACK_LOCKED:
                    printf("toast track locked \r\n");
                    NextState = FindTape;
                    MakeTransition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;
                    //                case ES_EXIT:
                    //                    ES_Timer_InitTimer(TF_SIT_AND_WAIT_TIMER, SIT_AND_WAIT_TIMEOUT);
                    //                    Event.EventType = ES_NO_EVENT;
                    //                    break;
            }
            break;

        case FindTape:
            Event = RunFindTapeSM(Event);
            switch (Event.EventType) {
                case ES_ENTRY:
                    Event.EventType = ES_NO_EVENT;
                    break;
                case BALL_DEPOSITED:
                    NextState = ExitVat;
//                    NextState = FindBeacon;
                    MakeTransition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case ExitVat:
            switch (Event.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(EXIT_VAT_TIMER, EXIT_TIMEOUT);
                    Toast_BothMtrSpeed(FORWARD_SPEED);
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless Event Catch
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:;
                    static uint8_t sequence = 0;
                    switch (Event.EventParam) {
                        case MANEUVER_TIMER:
                            switch (sequence) {
                            case 0:
                                Toast_StopMtr();
                                ES_Timer_InitTimer(MANEUVER_TIMER, 100);
                                sequence++;
                                break;
                            case 1:
                                Toast_StopMtr();
                                // translate backward
                                Toast_Pivot_FCW(90, FORWARD_SPEED);
                                sequence++;
                                break;
                            case 2:
                                Toast_StopMtr();
                                NextState = FindBeacon;
                                MakeTransition = TRUE;
                                sequence = 0; // Reset sequence counter
                                break;
                            default:
                                break;
                            }
                            Event.EventType = ES_NO_EVENT;
                            break;
                    }
                    break;

                case TAPE_DETECTED:
                    // Transition back to EvadeTape -> Will recall entry function perhaps? Might need test
                    ES_Timer_StopTimer(MANEUVER_TIMER);
                    TapeReading = Event.EventParam; // Store Tape Reading
                    NextState = EvadeTape;
                    MakeTransition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;
    }

    if (MakeTransition == TRUE) {
        RunToastHSM(EXIT_EVENT);
        CurrentState = NextState;
        RunToastHSM(ENTRY_EVENT);
    }

    ES_Tail();
    return Event;
}












