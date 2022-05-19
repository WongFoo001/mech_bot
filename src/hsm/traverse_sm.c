
/*
 * Track wires and traversal
 */

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "get_parallel_subhsm.h"
#include "motors.h"
#include "croissant_serial.h"
#include "wire_sense.h"

#define POWER 50
#define TIME_TO_TRANSLATE 150
#define BACKAROUND_TIME 200
#define PIVOT_OFFSET_TIME 25
#define OPTIMAL_DIST 25 // mm
#define STOP_FOR_SAMPLE_MS 500
#define FORE_TW_THRESH 95 // tune this
#define AFT_TW_THRESH 115 // tune this
#define CORRECTION_PIVOT 20 // deg
#define SLOP 5 // mm
#define ONTO_FACE_TIMER 75 // ms
#define CORNER_FACE_OFFSET 200

typedef enum {
    InitTTSubState,
    LockThatTrack,
    StopAndListen,
    STOPANDSAMPLE,
    STOPANDSAMPLE_2,
    STOPANDSAMPLE_3,
    FrontCCW,
    BackCW,
    Pivot_fwd,
    Pivot_back,
    Translate_fwd,
    Translate_back,
    TTSitAndWait,
    CORRECT_FWD,
    CORRECT_BACK,
} TTSubState_t;

static const char *StateNames[] = {
    "InitTTSubState",
    "LockThatTrack",
    "StopAndListen",
    "STOPANDSAMPLE",
    "STOPANDSAMPLE_2",
    "STOPANDSAMPLE_3",
    "FrontCCW",
    "BackCW",
    "Pivot_fwd",
    "Pivot_back",
    "Translate_fwd",
    "Translate_back",
    "TTSitAndWait",
    "CORRECT_FWD",
    "CORRECT_BACK",
};

static uint16_t* track_wires;

static struct lidar_sensors distances;

static TTSubState_t CurrentState = InitTTSubState;
static uint8_t prio;

uint8_t InitTTSubHSM(void) {
    ES_Event returnEvent;
    CurrentState = InitTTSubState;
    returnEvent = RunTTSubHSM(INIT_EVENT);
    // Initial Transition Event
    if (returnEvent.EventType == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

ES_Event RunTTSubHSM(ES_Event Event) {
    uint8_t make_transition = FALSE;
    static TTSubState_t next_state = InitTTSubState;

    ES_Tattle();

    switch (CurrentState) {
        case InitTTSubState:
            switch (Event.EventType) {
                case ES_INIT:
                    make_transition = TRUE;
                    next_state = StopAndListen;
                    Event.EventType = ES_NO_EVENT;
                    break;
                default:
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case TTSitAndWait:
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    printf("TT Sit and Wait\r\n");
                    Toast_StopMtr();
                    Event.EventType = ES_NO_EVENT;
                    break;

//                case ES_TIMEOUT:
//                    if (Event.EventParam == TT_SIT_AND_WAIT_TIMER) {
//                        next_state = StopAndListen;
//                        make_transition = TRUE;
//                        Event.EventType = ES_NO_EVENT;
//                    }
//                    break;
                default: 
                    Event.EventType = ES_NO_EVENT;
                    break;
            }
            break;

            // stop and listen to trackwires
        case StopAndListen:
            printf("stop and listen\r\n");
            switch (Event.EventType) {
                case ES_ENTRY:
                    // Stop motors
                    Toast_StopMtr();
                    ES_Timer_InitTimer(MANEUVER_TIMER, STOP_FOR_SAMPLE_MS);
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:;
                    static uint8_t ohShtCount = 0;
                    printf("tt listen timeout\r\n");
                    //                    if (Event.EventParam == MANEUVER_TIMER) {
                    //                        printf("stop and listen tome out maneuver timer\e\n");
                    // listen to trackwires
                    uint16_t* wires = getTrackWire();
                    printf("debug listen fore: %d \r\n aft: %d \r\n", wires[0], wires[1]);
                    // if we dont hear one one of the trackwires
                    if (!(wires[0] >= FORE_TW_THRESH && wires[1] >= AFT_TW_THRESH)) {
                        if (wires[0] < FORE_TW_THRESH && wires[1] < AFT_TW_THRESH) {
                            printf("HEY MAN, WE DONT HEAR SHIT \r\n");
                            printf("fore: %d \r\n aft: %d \r\n", wires[0], wires[1]);
                            next_state = Translate_back;
                            make_transition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                            ohShtCount++;
                            // we hear at least one trackwire
                        } else {
                            printf("we hear something \r\n");
                            // we dont hear the fore trackwire
                            if (wires[0] < FORE_TW_THRESH) {
                                printf("we hear somthing on aft \r\n");
                                // traverse aft
                                next_state = Translate_back;
                                make_transition = TRUE;
                                Event.EventType = ES_NO_EVENT;
                                // we dont hear the aft trackwire
                            } else {
                                printf("we hear something on fore\r\n");
                                // traverse fore 
                                next_state = Translate_fwd;
                                make_transition = TRUE;
                                Event.EventType = ES_NO_EVENT;

                            }
                        }
                        // we hear both trackwires
                    } else {
                        printf("both track wires go back to halt\r\n");
                        // leave and find tape
                        next_state = StopAndListen;
                        make_transition = TRUE;
                        Event.EventType = TRACK_LOCKED;
                        ohShtCount = 0; // Reset ohShtCount
                    }

                    //                    }
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



            // stop and listen to lidars
            // ENSURE YOU ARE REALLY ON A FACE
        case STOPANDSAMPLE:
            //printf("stop and sample \r\n");
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    //printf("stop and sample entry \r\n");
                    // Stop Motors
                    Toast_StopMtr();
                    // Enable UART
                    UART_EN = 1;
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                case DISTANCE_MEAS:
                    //printf("stop and sample dist meas \r\n");
                    // Store distance
                    distances = getDistancePacket();
                    // turn off uart enable
                    UART_EN = 0;
                    uint16_t fd = abs(distances.fore - distances.mid);
                    uint16_t ad = abs(distances.aft - distances.mid);

                    // check if all lidars are on the same face
                    if (!(fd <= ad + SLOP && fd >= ad - SLOP)) {

                        if (fd < ad) {
                            printf(" tt dist shimmy fwd \r\n");
                            // shimmy fwd
                            next_state = CORRECT_FWD;
                            make_transition = TRUE;
                            Event.EventType = ES_NO_EVENT;

                        } else {
                            printf("tt dist shimmy back \r\n");
                            // shimmy back
                            next_state = CORRECT_BACK;
                            make_transition = TRUE;
                            Event.EventType = ES_NO_EVENT;
                        }
                    } else {
                        printf("tt dist all on face \r\n");
                        Toast_StopMtr();
                        // all lidars are on the same face
                        next_state = StopAndListen;
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

        case STOPANDSAMPLE_2:
            printf("stop and sample 2 \r\n");
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    printf("stop and sample entry 2 \r\n");
                    // Stop Motors
                    Toast_StopMtr();
                    // Enable UART
                    UART_EN = 1;
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                case DISTANCE_MEAS:
                    printf("stop and sample dist meas 2\r\n");
                    // Store distance
                    distances = getDistancePacket();
                    // turn off uart enable
                    UART_EN = 0;

                    if (distances.mid >= MAX_DIST) { // Reached edge of vat
                        Toast_StopMtr();
                        next_state = Pivot_fwd;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    } else { // haven't reached edge of vat yet
                        next_state = Translate_fwd;
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

            // blindly translate forward
        case Translate_fwd:;
            // TRANSLATE BACKWARD
            switch (Event.EventType) {
                case ES_ENTRY:
                    Toast_BothMtrSpeed(POWER);
                    ES_Timer_InitTimer(MANEUVER_TIMER, TIME_TO_TRANSLATE);
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        //printf("fwd translate timer went off\r\n");
                        Toast_StopMtr();
                        make_transition = TRUE;
                        next_state = STOPANDSAMPLE_2;
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

        case Pivot_fwd:;
            // TRANSLATE BACKWARD
            switch (Event.EventType) {
                case ES_ENTRY:
                    Toast_BothMtrSpeed(POWER);
                    ES_Timer_InitTimer(MANEUVER_TIMER, PIVOT_OFFSET_TIME);
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:;
                    static uint8_t pivot_to_count_f = 0;
                    if (Event.EventParam == MANEUVER_TIMER) {
                        switch (pivot_to_count_f) {
                            case 0:
                                //printf("fwd pivot offset timer went off\r\n");
                                Toast_StopMtr();
                                Event.EventType = ES_NO_EVENT;
                                ES_Timer_InitTimer(MANEUVER_TIMER, 100);
                                pivot_to_count_f++;
                                break;
                            case 1:
                                //printf("Starting fwd pivot\r\n");
                                Toast_StopMtr();
                                Event.EventType = ES_NO_EVENT;
                                Toast_Pivot_FCCW(500, POWER);
                                pivot_to_count_f++;
                                break;
                            case 2:
                                //printf("fwd pivot finished\r\n");
                                Toast_StopMtr();
                                next_state = CORRECT_FWD;
                                make_transition = TRUE;
                                Event.EventType = ES_NO_EVENT;
                                pivot_to_count_f = 0;
                                break;

                        }

                    }
                    break;

                case BUMP_DANGER:
                    //printf("bumped in bump danger fwd pivot\r\n");
                    if (Event.EventParam > 0) {
                        ES_Timer_StopTimer(MANEUVER_TIMER);
                        next_state = CORRECT_FWD;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                        pivot_to_count_f = 0;
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

        case STOPANDSAMPLE_3:
            //printf("stop and sample \r\n");
            switch (Event.EventType) {
                    // Entry Event Response
                case ES_ENTRY:
                    //printf("stop and sample entry \r\n");
                    // Stop Motors
                    Toast_StopMtr();
                    // Enable UART
                    UART_EN = 1;
                    // Consume Entry Event
                    Event.EventType = ES_NO_EVENT;
                    break;

                case DISTANCE_MEAS:
                    //printf("stop and sample dist meas \r\n");
                    // Store distance
                    distances = getDistancePacket();
                    // turn off uart enable
                    UART_EN = 0;

                    if (distances.mid >= MAX_DIST) { // Reached edge of vat
                        Toast_StopMtr();
                        next_state = Pivot_back;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                    } else { // haven't reached edge of vat yet
                        next_state = Translate_back;
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

            // blindly translate backward
        case Translate_back:;
            // TRANSLATE BACKWARD
            switch (Event.EventType) {
                case ES_ENTRY:
                    Toast_BothMtrSpeed(-POWER);
                    ES_Timer_InitTimer(MANEUVER_TIMER, TIME_TO_TRANSLATE);
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        //printf("back translate timer went off\r\n");
                        Toast_StopMtr();
                        make_transition = TRUE;
                        next_state = STOPANDSAMPLE_3;
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

        case Pivot_back:;
            // TRANSLATE BACKWARD
            switch (Event.EventType) {
                case ES_ENTRY:
                    Toast_BothMtrSpeed(-POWER);
                    ES_Timer_InitTimer(MANEUVER_TIMER, PIVOT_OFFSET_TIME);
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:;
                    static uint8_t pivot_to_count_b = 0;
                    if (Event.EventParam == MANEUVER_TIMER) {
                        switch (pivot_to_count_b) {
                            case 0:
                                //printf("back pivot offset timer went off\r\n");
                                Toast_StopMtr();
                                Event.EventType = ES_NO_EVENT;
                                ES_Timer_InitTimer(MANEUVER_TIMER, 100);
                                pivot_to_count_b++;
                                break;
                            case 1:
                                //printf("Starting back pivot\r\n");
                                Toast_StopMtr();
                                Event.EventType = ES_NO_EVENT;
                                Toast_Pivot_BCW(500, POWER);
                                pivot_to_count_b++;
                                break;
                            case 2:
                                //printf("back pivot finished\r\n");
                                Toast_StopMtr();
                                next_state = CORRECT_BACK;
                                make_transition = TRUE;
                                Event.EventType = ES_NO_EVENT;
                                pivot_to_count_b = 0;
                                break;
                        }
                    }
                    break;

                case BUMP_DANGER:
                    printf("bumped in bump danger fwd pivot\r\n");
                    if (Event.EventParam == 8) { // Back left bumper hit
                        ES_Timer_StopTimer(MANEUVER_TIMER);
                        next_state = CORRECT_BACK;
                        make_transition = TRUE;
                        Event.EventType = ES_NO_EVENT;
                        pivot_to_count_b = 0;
                    }
                    else if (Event.EventParam > 16){ // Back left and center hit, do a lil pivot
                        next_state = StopAndListen;
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

        case CORRECT_FWD:;
            static uint8_t last_pushf = 0;
            // translate fwd
            switch (Event.EventType) {
                case ES_ENTRY:
                    Toast_StopMtr();
                    Event.EventType = ES_NO_EVENT;
                    UART_EN = 1;
                    break;

                case DISTANCE_MEAS:
                    printf("CORRECT FWD dist meas\r\n");
                    UART_EN = 0;
                    Toast_StopMtr();
                    distances = getDistancePacket();
                    if (distances.fore >= MAX_DIST || distances.mid >= MAX_DIST || distances.aft >= MAX_DIST) {
                        printf("correct fwd not all lidar on face\r\n");
                        Toast_BothMtrSpeed(POWER);
                        ES_Timer_InitTimer(MANEUVER_TIMER, ONTO_FACE_TIMER);
                    } else {
                        printf("correct fwd all lidar on face tranition to stop and listen\r\n");
                        Toast_BothMtrSpeed(POWER);
                        ES_Timer_InitTimer(MANEUVER_TIMER, CORNER_FACE_OFFSET);
                        last_pushf = 1;
//                        next_state = TTSitAndWait;
//                        make_transition = TRUE;
                    }
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        if (last_pushf == 0) {
                            Toast_StopMtr();
                            UART_EN = 1;
                        } else {
                            next_state = StopAndListen;
//                            next_state = TTSitAndWait;
                            make_transition = TRUE;
                            last_pushf = 0;
                        }

                    }
                    Event.EventType = ES_NO_EVENT;
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



        case CORRECT_BACK:;
            static uint8_t last_pushb = 0;
            // translate back
            switch (Event.EventType) {
                case ES_ENTRY:
                    Toast_StopMtr();
                    Event.EventType = ES_NO_EVENT;
                    UART_EN = 1;
                    break;

                case DISTANCE_MEAS:
                    printf("Correct back dist meas\r\n");
                    UART_EN = 0;
                    Toast_StopMtr();
                    distances = getDistancePacket();
                    if (distances.fore >= MAX_DIST || distances.mid >= MAX_DIST || distances.aft >= MAX_DIST) {
                        printf("correct back not all lidar on face\r\n");
                        Toast_BothMtrSpeed(-POWER);
                        ES_Timer_InitTimer(MANEUVER_TIMER, ONTO_FACE_TIMER);
                    } else {
                        printf("correct back all lidar on face tranition to ball deploy\r\n");
                        Toast_BothMtrSpeed(-POWER);
                        ES_Timer_InitTimer(MANEUVER_TIMER, CORNER_FACE_OFFSET);
                        last_pushb = 1;
//                        next_state = TTSitAndWait;
//                        make_transition = TRUE;
                    }
                    Event.EventType = ES_NO_EVENT;
                    break;

                case ES_TIMEOUT:
                    if (Event.EventParam == MANEUVER_TIMER) {
                        if (last_pushb == 0) {
                            Toast_StopMtr();
                            UART_EN = 1;
                        } else {
//                            next_state = TTSitAndWait;
                            next_state = StopAndListen;
                            make_transition = TRUE;
                            last_pushb = 0;
                        }
                    }
                    Event.EventType = ES_NO_EVENT;
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
    }

    if (make_transition == TRUE) {
        RunTTSubHSM(EXIT_EVENT);
        CurrentState = next_state;
        RunTTSubHSM(ENTRY_EVENT);
    }

    ES_Tail();
    return Event;
}
