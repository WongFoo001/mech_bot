#include "wire_sense.h"
#include "xc.h"
#include "toast_HSM.h"

static uint8_t P;

#define WIRE_TICKS 100

uint16_t* getTrackWire(void){
    static uint16_t ret[2]; // important that this is static so that an array is returned
    uint16_t fore_wire_sense = AD_ReadADPin(FORE_WIRE);
    uint16_t aft_wire_sense = AD_ReadADPin(AFT_WIRE);
    
    printf("fore tw:%d \r\n aft tw:%d\r\n", fore_wire_sense, aft_wire_sense);
    
    ret[0] = fore_wire_sense;
    ret[1] = aft_wire_sense;
    
    return ret;
}

uint8_t InitWireService(uint8_t Priority){
	ES_Event Event;
	P = Priority;
    //ES_Timer_InitTimer(WIRE_SERVICE_TIMER, WIRE_TICKS);
	
	Event.EventType = ES_INIT;
	if(ES_PostToService(P, Event) == TRUE){
		return TRUE;
	}else{
		return FALSE;
	}
}

uint8_t PostWireService(ES_Event E){
	return ES_PostToService(P, E);
}

ES_Event RunWireService(ES_Event E){
    static uint8_t last_wire = 0x00;
	uint8_t returnVal = FALSE;
	
    ES_Event WireEvent;
    WireEvent.EventType = ES_NO_EVENT; // Assume no event by default
    ES_EventTyp_t curFloor;
    switch (E.EventType) {
        case ES_INIT:
            break;

        case ES_TIMERACTIVE:
        case ES_TIMERSTOPPED:
            break;

        case ES_TIMEOUT:;
            //ES_Timer_InitTimer(WIRE_SERVICE_TIMER, WIRE_TICKS);
            uint16_t fore_wire_sense = AD_ReadADPin(FORE_WIRE);
            uint16_t aft_wire_sense = AD_ReadADPin(AFT_WIRE);
            
            printf("fore: %d || aft: %d\r\n", fore_wire_sense, aft_wire_sense);

            uint16_t wires[] = {fore_wire_sense, aft_wire_sense};
            int i = 0;
            uint8_t ret = 0x00;
            while(i < 2){
                if(wires[i] > WIRE){
                    ret |= (1 << i);
                }
                i++;
            }

            if(ret != last_wire){
                WireEvent.EventType = WIRE_CHANGE;
                WireEvent.EventParam = ret;
                returnVal = TRUE;
                last_wire = ret;
                #ifndef EVENTCHECKER_TEST // FOR TEST HARNESS
                PostToastHSM(WireEvent);
                #else
                SaveEvent(thisEvent);
                #endif
            }
    }
	return WireEvent;
}
