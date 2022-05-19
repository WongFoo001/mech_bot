
#include "croissant_serial.h"


#define accepted 

#define F_PB (BOARD_GetPBClock())

// you may want to make this smaller - 64 maybe, 8 might be good
#define CROISSANT_QUEUESIZE 7

#define OUTPUT 0
#define INPUT 1

static uint8_t AddingToTransmit = FALSE;
static uint8_t GettingFromReceive = FALSE;
static uint8_t Receiving = TRUE;
static uint8_t TransmitCollisionOccured = FALSE;
static uint8_t ReceiveCollisionOccured = FALSE;

static uint8_t TransmitBuffer[CROISSANT_QUEUESIZE];
static uint32_t TransmitHead = 0;
static uint32_t TransmitTail = 0;

static uint8_t ReceiveBuffer[CROISSANT_QUEUESIZE];
static uint8_t ReceiveHead = 0;
static uint8_t ReceiveTail = 0;
static uint8_t x;

char CROISSANT_Init(void)
{
    U2MODE = 0; // Clear the control registers for the UART
    U2STA = 0;

    // calculate BRG value
    unsigned int brgValue = F_PB;
    brgValue >>= 3;
    brgValue /= 115200;
    brgValue++;
    brgValue >>= 1;
    brgValue--;
    U2BRG = brgValue; // set the baud rate to 115200
    // have interrupts occur whenever RX is not empty
    U2STAbits.URXISEL = 0;
    U2STAbits.UTXISEL = 0;

    IPC8bits.U2IP = 4; //set the interrupt priority

    // we now enable the module and both RX and TX
    U2MODEbits.ON = 1; // turn on uart itself
    U2STAbits.UTXEN = 1; // turn on TX
    U2STAbits.URXEN = 1; // turn on RX
    // enable the interrupts
    IEC1bits.U2RXIE = 1;
    IEC1bits.U2TXIE = 1;
    
    UART_EN_TRIS = OUTPUT;
    UART_EN = 0;
    
    return TRUE;
}

void PutCroissant(char ch)
{
    if (TransmitTail - TransmitHead != CROISSANT_QUEUESIZE) {
        AddingToTransmit = TRUE;
        TransmitBuffer[TransmitTail] = ch;
        TransmitTail = (TransmitTail + 1) % CROISSANT_QUEUESIZE;
        AddingToTransmit = FALSE;
        if (U2STAbits.TRMT) {
            IFS1bits.U2TXIF = 1;
        }
        //re-enter the interrupt if we removed a character while getting another one
        if (TransmitCollisionOccured) {
            IFS1bits.U2TXIF = 1;
            TransmitCollisionOccured = FALSE;

        }

    }
}

char GetCroissant(void)
{
    char ch;
    if (ReceiveTail == ReceiveHead) {
        ch = 0;
    } else {
        GettingFromReceive = TRUE;
        ch = ReceiveBuffer[ReceiveHead];
        ReceiveHead = (ReceiveHead+1) % CROISSANT_QUEUESIZE;
        GettingFromReceive = FALSE;
    }
    //re-enter the interrupt if we added a character while transmitting another one
    if (ReceiveCollisionOccured) {
        IFS1bits.U2RXIF = 1;
        ReceiveCollisionOccured = FALSE;
    }
    return ch;
}

void _mon_putcroissant(char c)
{
    PutCroissant(c);
}

void _mon_putscroissant(const char* s)
{
    int i;
    for (i = 0; s[i] != '\0'; i++)
        PutCroissant(s[i]);
}

int _mon_getcroissant(int CanBlock)
{
    if (ReceiveTail == ReceiveHead)
        return -1;
    return GetCroissant();
}

char IsReceiveEmptyC(void)
{
    if (ReceiveTail == ReceiveHead)
        return TRUE;
    return FALSE;
}

char IsReceiveFullC(void){
    if ((ReceiveTail + 1) % CROISSANT_QUEUESIZE == ReceiveHead){
//        printf("RECEIVE IS FULL\r\n");
        return TRUE;
    }
    return FALSE;
}

char IsTransmitEmptyC(void)
{
    if (TransmitTail == TransmitHead)
        return TRUE;
    return FALSE;
}

struct lidar_sensors getDistancePacket(void){
    //printf("Distance Packet: %u\r\n", distancePacket);
    return lidars;
}

// This shifts the values for es framework param
void shifter(char place, uint8_t value){
//    printf("val: %u | ", value);
    uint32_t ret = 0;
    // garbage management
    if(value > MAX_DIST){
        value = MAX_DIST;
    }
    
    // Check for low value issue 
    if(place == 0){
        // check for zeros
        lidars.aft = value>5 ? value:lidars.aft;
    }
    if(place == 1){
        lidars.mid = value>5 ? value:lidars.mid;
    }
    if(place == 2){
        lidars.fore = value>5 ? value:lidars.fore;
    }
}

// clear the buffer and turn to es_events
ES_Event EatCroissants(void){
    ES_Event E;
    E.EventType = ES_NO_EVENT; // Default assume no event
    // package for the es framework
    if (IsReceiveFullC()){
//            printf("yummy croissants\r\n");
        int idx = 0;
        int FULL_FLAG = 0;
        uint32_t dists = 0;
        idx = 0;
//        while(idx < CROISSANT_QUEUESIZE){
//            printf("%u ", ReceiveBuffer[idx]);
//            idx++;
//        }
        idx = 0;
        while(idx < CROISSANT_QUEUESIZE-1){
            uint8_t val = ReceiveBuffer[idx];
            //     fore        mid         aft
            if((val == 0 || val == 1 || val == 2) && FULL_FLAG < 3){
                // correctly encode bytes for param
                shifter(val, ReceiveBuffer[idx + 1]);
                idx++;
                FULL_FLAG++;
            }
            idx++;
        }
        ReceiveTail = ReceiveHead;
        dists = TRUE; // Set distance packet
        E.EventType = DISTANCE_MEAS;
        E.EventParam = dists; // param is only 16 bits large
        if (UART_EN){
            PostToastHSM(E);
            //printf("croissant publishing\r\n");
        }
        // THIS IS IMPORTANT AND OPENS THE BUFFER BACK UP FOR RECEIVING
        Receiving = TRUE;
        }
        
    return E;
}

void __ISR(_UART2_VECTOR) IntUart2Handler(void)
{
    if (IFS1bits.U2RXIF) {
        IFS1bits.U2RXIF = 0;
        // turn off receiving when buffer is full
//        if(Receiving == TRUE){
            if (!GettingFromReceive) {
                ReceiveBuffer[ReceiveTail] = U2RXREG;
                x = ReceiveBuffer[ReceiveTail];
                //printf("x: %X \r\n", x);
                //Put this on a buffer or value of our own to be used for transmitting to the rest of the
                ReceiveTail = (ReceiveTail+1) % CROISSANT_QUEUESIZE;
//                if(IsReceiveFullC() == TRUE){
//                    // THIS LOCKS OUT THE RECEIVE BUFFER WHEN FULL
////                    printf("RECEIVE LOCKED\r\n");
//                    Receiving = FALSE;
//                }
            } else {
                //acknowledge we have a collision and return
                ReceiveCollisionOccured = TRUE;
            }
//        }else{
//            uint8_t c = U2RXREG;
//        }
    }

    if (IFS1bits.U2TXIF) { // Don't think we'll need since it's one way
        IFS1bits.U2TXIF = 0;
        if ((!AddingToTransmit)) {
            if (TransmitTail != TransmitHead) {
                U2TXREG = TransmitBuffer[TransmitHead];
                TransmitHead = (TransmitHead + 1) % CROISSANT_QUEUESIZE;
            }
        } else {
            //acknowledge we have a collision and return
            TransmitCollisionOccured = TRUE;
        }

    }

}

