// THE RIGHT TOASTER FILE
/*
 * File:   toaster.c
 * Author: juwewong
 *
 * Created on November 16, 2021, 2:14 PM
 */
#include "xc.h"
#include "toaster.h"
#include "BOARD.h"
#include "pwm.h"
#include "AD.h"

#define OUTPUT 0
#define INPUT  1

#define FORWARD  1
#define BACKWARD 0
#define TOAST_MAX_SPEED 100

#define RIGHT_MTR_PWM PWM_PORTY10 
#define LEFT_MTR_PWM  PWM_PORTY12

// Signal Directions
#define RIGHT_MTR_DIR_TRIS TRISGbits.TRISG6
#define RIGHT_MTR_INV_TRIS TRISGbits.TRISG7
#define LEFT_MTR_DIR_TRIS  TRISFbits.TRISF6
#define LEFT_MTR_INV_TRIS  TRISDbits.TRISD4

// Port References
#define RIGHT_MTR_DIR LATGbits.LATG6 // X05
#define RIGHT_MTR_INV LATGbits.LATG7 // X07
#define LEFT_MTR_DIR  LATFbits.LATF6 // X08
#define LEFT_MTR_INV  LATDbits.LATD4 // X11

/*******************************************************************************
 * PUBLIC FUNCTION                                                *
 ******************************************************************************/
void Toaster_Init(void){
    // Initialize Motor PWM Signals
    PWM_Init();
    PWM_SetFrequency(PWM_2KHZ);
    PWM_AddPins(RIGHT_MTR_PWM | LEFT_MTR_PWM);
    
    // Motor signal direction initializations
    RIGHT_MTR_DIR_TRIS = OUTPUT;
    RIGHT_MTR_INV_TRIS = OUTPUT;
    LEFT_MTR_DIR_TRIS  = OUTPUT;
    LEFT_MTR_INV_TRIS  = OUTPUT;
    
    // Motor signal value initializations
    RIGHT_MTR_DIR = FORWARD;
    RIGHT_MTR_INV = BACKWARD;
    LEFT_MTR_DIR  = FORWARD;
    LEFT_MTR_INV  = BACKWARD;
}

char Toast_LeftMtrSpeed(char newSpeed){
    if ((newSpeed < -TOAST_MAX_SPEED) || (newSpeed > TOAST_MAX_SPEED)) {
        return (ERROR);
    }
    if (newSpeed < 0) {
        LEFT_MTR_DIR = BACKWARD;
        newSpeed = newSpeed * (-1); // set speed to absolute value
    } else {
        LEFT_MTR_DIR = FORWARD;
    }
    LEFT_MTR_INV = ~LEFT_MTR_DIR;
    if (PWM_SetDutyCycle(LEFT_MTR_PWM, newSpeed * (MAX_PWM / TOAST_MAX_SPEED)) == ERROR) {
        //printf("ERROR: setting channel 1 speed!\n");
        return (ERROR);
    }
    return (SUCCESS);
}

char Toast_RightMtrSpeed(char newSpeed){
    if ((newSpeed < -TOAST_MAX_SPEED) || (newSpeed > TOAST_MAX_SPEED)) {
        return (ERROR);
    }
    if (newSpeed < 0) {
        RIGHT_MTR_DIR = BACKWARD;
        newSpeed = newSpeed * (-1); // set speed to absolute value
    } else {
        RIGHT_MTR_DIR = FORWARD;
    }
    RIGHT_MTR_INV = ~RIGHT_MTR_DIR;
    if (PWM_SetDutyCycle(RIGHT_MTR_PWM, newSpeed * (MAX_PWM / TOAST_MAX_SPEED)) == ERROR) {
        //printf("ERROR: setting channel 1 speed!\n");
        return (ERROR);
    }
    return (SUCCESS);
}