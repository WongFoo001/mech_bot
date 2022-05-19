
#include "ToastEventChecker.h"

#define SEE 750
#define NO_SEE 650

uint8_t ToastCheckBeacon(void){
    static ES_EventTyp_t lastEvent = LOST_BEACON;
	ES_EventTyp_t curEvent = lastEvent;
	ES_Event hsmEvent;
    hsmEvent.EventType = ES_NO_EVENT; // By default, pass no event
	uint8_t returnVal = FALSE;

    if (AD_IsNewDataReady()){
        uint16_t beacon_sens = AD_ReadADPin(BEACON_IN);
        //printf("beacon reading: %d \r\n", beacon_sens);
        if (lastEvent == LOST_BEACON){
            if (beacon_sens > SEE){
                curEvent = SEE_BEACON;
            }
        }
        else{ // lastEvent == SEE_BEACON;
            if (beacon_sens < NO_SEE){
                curEvent = LOST_BEACON;
            }
        }
        
        if(curEvent != lastEvent){
            hsmEvent.EventType = curEvent;
            hsmEvent.EventParam = beacon_sens;
            returnVal = TRUE;
            lastEvent = curEvent;
            //printf("Posting Beacon Event to PostToastHSM");
            PostToastHSM(hsmEvent);
        }
    }
	return (returnVal);
}

uint8_t ToastCheckUART(void){
//    static ES_EventTyp_t lastEvent = LOST_BEACON;
	ES_EventTyp_t curEvent;
	ES_Event hsmEvent;
    hsmEvent.EventType = ES_NO_EVENT; // By default, pass no event
	uint8_t returnVal = FALSE;
    
    hsmEvent = EatCroissants();
    returnVal = TRUE;
    
	return (returnVal);
}

uint8_t ToastCheckHall(void){
    //printf("TOAST CHECK HALL\r\n");
    static ES_EventTyp_t lastEvent = NO_HALL;
	ES_EventTyp_t curEvent = lastEvent;
	ES_Event hsmEvent;
    hsmEvent.EventType = ES_NO_EVENT; // By default, pass no event
	uint8_t returnVal = FALSE;

    if (HALL_SENSOR_SIG){
        //printf("DETECTING HALL\r\n");
        curEvent = NO_HALL;
    }
    else{
        //printf("NOT DETECTING HALL\r\n");
        curEvent = HALL_DETECTED;
    }
    
    if(curEvent != lastEvent){
        if (curEvent == HALL_DETECTED){
            printf("HALL EVENT POSTED!\r\n");
        }
        else{
            printf("HALL LOST!\r\n");
        }
        
        hsmEvent.EventType = curEvent;
        hsmEvent.EventParam = 0;
        returnVal = TRUE;
        
        //printf("Posting Beacon Event to PostToastHSM");
        PostToastHSM(hsmEvent);
    }
    
    lastEvent = curEvent;
	return (returnVal);
}
