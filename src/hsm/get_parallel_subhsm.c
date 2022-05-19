
#include "get_parallel_subhsm.h"

#define POWER 60
#define TRANSLATION_TIME_MS 100
#define TIME_TO_GET_ALL_LIDARS_ON_A_FACE 3000
#define TINY_BACKUP_MS 180
#define LIL_NUDGE_MS 100    
#define OPTIMAL_DIST 15 // mm
#define ZPT_TIME 100
#define MAX_DIST 200 // mm
#define BALANCE_DEG 5
#define SLOP 8
#define WRONG_POS_CHECK 30

typedef enum {
    InitGPSubState,
    GPSitAndWait,
    TinyBackup,
    InitialZeroPt,
    SecondZeroPt,
    STOPANDSAMPLE_1,
    STOPANDSAMPLE_2,
    STOPANDSAMPLE_3,
    STOPANDSAMPLE_4,
    STOPANDSAMPLE_5,
    LilNudge,
    FrontCCW,
    BackCW,
    ShimmyBF,
    ShuffleIn,
    WrongPos,
} GPSubState_t;

static const char *StateNames[] = {
    "InitGPSubState",
    "GPSitAndWait",
    "TinyBackup",
    "InitialiZeroPt",
    "SecondZeroPt",
    "STOPANDSAMPLE_1",
    "STOPANDSAMPLE_2",
    "STOPANDSAMPLE_3",
    "STOPANDSAMPLE_4",
    "STOPANDSAMPLE_5",
    "LilNudge",
    "FrontCCW",
    "BackCW",
    "ShimmyBF",
    "ShuffleIn",
    "WrongPos",
};

static struct lidar_sensors distances;

static GPSubState_t CurrentState = InitGPSubState;
static uint8_t prio;

uint8_t InitGPSubHSM(void) {
    ES_Event returnEvent;
    CurrentState = InitGPSubState;
    returnEvent = RunGPSubHSM(INIT_EVENT);
    // Initial Transition Event
    if (returnEvent.EventType == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

ES_Event RunGPSubHSM(ES_Event Event) {
    uint8_t make_transition = FALSE;
    GPSubState_t next_state;

    ES_Tattle();

    // Top Level State Switch Statement
    switch (CurrentState) {
            // Initialization State
        case InitGPSubState:
            switch (Event.EventType) {
                case ES_INIT:
                    next_state = TinyBackup;
                    make_transition = TRUE;
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

            // Initial ZeroPT State
            //        case GPSitAndWait:
            //            switch (Event.EventType) {
            //                    // Entry Event Response
            //                case ES_ENTRY:
            //                    printf("GP Sit and Wait\r\n");
            ////                    Toast_StopMtr();
            //                    Event.EventType = ES_NO_EVENT;
            //                    break;
            //                case ES_TIMEOUT:
            //                    if (Event.EventParam == GP_SIT_AND_WAIT_TIMER) {
            //                        next_state = TinyBackup;
            //                        make_transition = TRUE;
            //                        Event.EventType = ES_NO_EVENT;
            //                    }
            //                    break;
            //            }
            //            break;

            // Tiny Backup State
        case TinyBackup:
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    // Set Backup Timer
                    ES_Timer_InitTimer(MANEUVER_TIMER, TINY_BACKUP_MS);
                    // Set Motors to reverse
                    Toast_BothMtrSpeed(-POWER);
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Timeout event catcher
                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        printf("Tiny Backup going to InitZeroPt\r\n");
                        next_state = InitialZeroPt;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;

            // Initial ZeroPT State
        case InitialZeroPt:
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    // Start ZeroPT turn
                    ES_Timer_InitTimer(MANEUVER_TIMER, ZPT_TIME);
                    Toast_ZeroPt_CW(POWER);
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Timeout event catcher
                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        printf("Init ZeroPt finished, going to SS_1\r\n");
                        next_state = STOPANDSAMPLE_1;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;

                case BUMP_DANGER:
                    if (Event.EventParam > 0) {
                        printf("Bump felt, going back to TinyBackup\r\n");
                        next_state = TinyBackup;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;

        case STOPANDSAMPLE_1:;
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    // Stop Motors
                    Toast_StopMtr();
                    // Enable UART
                    UART_EN = 1;
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                case DISTANCE_MEAS:
                    // Store distance
                    distances = getDistancePacket();
                    // turn off uart enable
                    UART_EN = 0;

                    //printf("Distances: f%d, m%d, a%d\r\n", distances.fore, distances.mid, distances.aft);

                    // If mid not found keep zero point turning
                    if (distances.mid >= MAX_DIST) {
                        //printf("Going to back to InitZeroPt\r\n");
                        next_state = InitialZeroPt;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    } else { // Mid is found
                        //printf(" Mid Found, Moving onto SS_2\r\n");
                        next_state = STOPANDSAMPLE_2;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case STOPANDSAMPLE_2:;
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    // Stop Motors
                    Toast_StopMtr();
                    // Enable UART
                    UART_EN = 1;
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                case DISTANCE_MEAS:
                    // Store distance
                    distances = getDistancePacket();
                    // turn off uart enable
                    UART_EN = 0;

                    //printf("Distances: f%d, m%d, a%d\r\n", distances.fore, distances.mid, distances.aft);

                    // If fore not found keep pivot turning
                    if (distances.fore >= MAX_DIST) {
                        //printf("Fore not found, pivoting slightly\r\n");
                        next_state = FrontCCW;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    } else { // Fore is found
                        //printf("Fore found, going to SS_3\r\n");
                        next_state = STOPANDSAMPLE_3;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case FrontCCW:
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    Toast_P_FCCW(BALANCE_DEG, POWER);
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Timeout event catcher
                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        printf("Front pivot finished\r\n");
                        next_state = STOPANDSAMPLE_2;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;

        case STOPANDSAMPLE_3:;
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    // Stop Motors
                    Toast_StopMtr();
                    // Enable UART
                    UART_EN = 1;
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                case DISTANCE_MEAS:
                    // Store distance
                    distances = getDistancePacket();
                    // turn off uart enable
                    UART_EN = 0;

                    //printf("Distances: f%d, m%d, a%d\r\n", distances.fore, distances.mid, distances.aft);

                    // Make sure fore and mid still exist
                    if (distances.mid < MAX_DIST && distances.fore < MAX_DIST) {
                        // If mid greater than fore
                        if (distances.mid > (distances.fore + SLOP)) {
                            //printf("Mid > Fore, ZeroPt_CW\r\n");
                            next_state = SecondZeroPt;
                            make_transition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                        } else if (distances.fore > (distances.mid + SLOP)) { //fore greater than mid
                            //printf("Fore > Mid, ZeroPt_CCW\r\n");
                            next_state = SecondZeroPt;
                            make_transition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                        } else if (((distances.fore - distances.mid) < (SLOP * 2)) || ((distances.mid - distances.fore) < (SLOP * 2))) { // Mid and fore are parallel enough
                            //printf("Fore and mid parallelized\r\n");
                            next_state = ShimmyBF;
                            make_transition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                        }
                    } else { // Return back to SS2 (Super Saiyan 2)
                        //printf("Fore or mid lost, go back to SS_1\r\n");
                        next_state = STOPANDSAMPLE_1;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }

                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case SecondZeroPt:
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    // Start ZeroPT turn
                    ES_Timer_InitTimer(MANEUVER_TIMER, ZPT_TIME);
                    if (distances.mid > (distances.fore + SLOP)) {
                        Toast_ZeroPt_CW(POWER);
                    } else { // distances.fore > (distances.mid + SLOP)
                        Toast_ZeroPt_CCW(POWER);
                    }
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Timeout event catcher
                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        printf("Second ZeroPt Finished\r\n");
                        next_state = STOPANDSAMPLE_3;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;

        case STOPANDSAMPLE_4:;
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    // Stop Motors
                    Toast_StopMtr();
                    // Enable UART
                    UART_EN = 1;
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                case DISTANCE_MEAS:
                    // Store distance
                    distances = getDistancePacket();
                    // turn off uart enable
                    UART_EN = 0;

                    //printf("Distances: f%d, m%d, a%d\r\n", distances.fore, distances.mid, distances.aft);

                    // Make sure fore and mid still exist
                    if (distances.aft >= MAX_DIST) {
                        printf("Aft still not seen, shimmying more\r\n");
                        next_state = ShimmyBF;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    } else { // All 3 Parallel Achieved!
                        printf("ALL 3 PARALLELIZED\r\n");
                        next_state = ShuffleIn;
                        //                        next_state = BigPHalt; 
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

            // Shimmies back and forth to get all sensors on face
        case ShimmyBF:
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    Toast_BothMtrSpeed(POWER);
                    // Set timer that correlates to 1in of travel
                    ES_Timer_InitTimer(MANEUVER_TIMER, TRANSLATION_TIME_MS);
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        next_state = STOPANDSAMPLE_4;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;

        case STOPANDSAMPLE_5:;
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    // Stop Motors
                    Toast_StopMtr();
                    // Enable UART
                    UART_EN = 1;
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                case DISTANCE_MEAS:
                    // Store distance
                    distances = getDistancePacket();
                    // turn off uart enable
                    UART_EN = 0;

                    //printf("Distances: f%d, m%d, a%d\r\n", distances.fore, distances.mid, distances.aft);

                    // Make sure fore and mid still exist
                    if (distances.aft >= MAX_DIST) {
                        printf("Aft lost giving it a lil nudge\r\n");
                        next_state = LilNudge;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    } else if (distances.mid <= OPTIMAL_DIST + SLOP) {
                        if ((distances.aft - distances.mid) > WRONG_POS_CHECK) {
                            next_state = TinyBackup;
                            make_transition = TRUE;
                            Event.EventType = SEARCH_TRACKWIRE; // LEAVE FOR TRAVERSE
                        }
                        printf("Toast within distance margin!\r\n");
                        next_state = TinyBackup;
                        make_transition = TRUE;
                        Event.EventType = SEARCH_TRACKWIRE; // LEAVE FOR TRAVERSE
                        //                        Event.EventType = ES_NO_EVENT;
                    } else { // All 3 Parallel Achieved!
                        printf("Toast still too far out\r\n");
                        next_state = ShuffleIn;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }

                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

            // the electric slide 
        case ShuffleIn:
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    printf("SHUFFLE SHUFFLE \r\n");
                    ES_Timer_InitTimer(MANEUVER_TIMER, 1);
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:;
                    static int sequence = 0;
                    if (Event.EventParam == MANEUVER_TIMER) {
                        switch (sequence) {
                            case 0:
                                Toast_StopMtr();
                                // zero pt turn cw
                                ES_Timer_InitTimer(MANEUVER_TIMER, 100);
                                sequence++;
                                Toast_ZeroPt_CW(POWER);
                                break;
                            case 1:
                                Toast_StopMtr();
                                ES_Timer_InitTimer(MANEUVER_TIMER, 100);
                                sequence++;
                                break;
                            case 2:
                                Toast_StopMtr();
                                // translate backward
                                ES_Timer_InitTimer(MANEUVER_TIMER, 250);
                                sequence++;
                                Toast_BothMtrSpeed(-POWER);
                                break;
                            case 3:
                                Toast_StopMtr();
                                ES_Timer_InitTimer(MANEUVER_TIMER, 100);
                                sequence++;
                                break;
                            case 4:
                                Toast_StopMtr();
                                // zero pt turn ccw
                                ES_Timer_InitTimer(MANEUVER_TIMER, 105);
                                sequence++;
                                Toast_ZeroPt_CCW(POWER);
                                break;
                            case 5:
                                Toast_StopMtr();
                                // zero pt turn cw
                                ES_Timer_InitTimer(MANEUVER_TIMER, 100);
                                sequence++;
                                break;
                            case 6:
                                Toast_StopMtr();
                                // translate forward
                                ES_Timer_InitTimer(MANEUVER_TIMER, 257);
                                sequence++;
                                Toast_BothMtrSpeed(POWER);
                                break;
                            case 7:
                                Toast_StopMtr();
                                // zero pt turn cw
                                ES_Timer_InitTimer(MANEUVER_TIMER, 100);
                                sequence++;
                                break;
                            case 8:
                                sequence = 0;
                                Toast_StopMtr();
                                next_state = STOPANDSAMPLE_5;
                                make_transition = TRUE;
                                printf("SHUFFLE SHUFFLE FINISHED\r\n");
                                break;
                            default:
                                break;
                        }
                    }
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

            // LilNudge State
        case LilNudge:
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    // Set Backup Timer
                    ES_Timer_InitTimer(MANEUVER_TIMER, LIL_NUDGE_MS);
                    // Set Motors to reverse
                    Toast_BothMtrSpeed(POWER);
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Useless event catchers
                case ES_TIMERACTIVE:
                case ES_TIMERSTOPPED:
                case ES_NO_EVENT:
                case ES_EXIT:
                    Event.EventType = ES_NO_EVENT;
                    break;

                    // Timeout event catcher
                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        printf("Lil Nudge going to STOPANDSAMPLE_5\r\n");
                        next_state = STOPANDSAMPLE_5;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;

            // Initial ZeroPT State
        case WrongPos:
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    printf("Wrong Pos found - evasive maneuver\r\n");
                    // Set Backup Timer
                    Toast_Pivot_BCW(50, POWER);
                    break;
                    // Timeout event catcher
                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        printf("FINISHED WRONG POS EVASIVE MANEUVER\r\n");
                        next_state = STOPANDSAMPLE_1;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;
    }
    if (make_transition == TRUE) {
        RunGPSubHSM(EXIT_EVENT);
        CurrentState = next_state;
        RunGPSubHSM(ENTRY_EVENT);
    }

    ES_Tail();
    return Event;
}