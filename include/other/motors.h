/* 
 * File:   toaster.h
 * Author: juwewong
 *
 * Created on November 16, 2021, 2:04 PM
 */
#ifndef MOTORS_H
#define MOTORS_H

#include "ES_Configure.h"
#include "xc.h"
// #include "toaster.h" obsolete -> all transitioned to motors.h/c
#include "BOARD.h"
#include "pwm.h"
#include "AD.h"
#include "stdio.h"
#include "ES_Framework.h"

#define WHEELDIAMETER 90 // mm




void Toaster_Init(void);

int deg_time(int deg, char rate, int off_set);

char Toast_LeftMtrSpeed(char newSpeed);

char Toast_RightMtrSpeed(char newSpeed);

// set speed of both motors
char Toast_BothMtrSpeed(char newSpeed);

// stop both motors
char Toast_StopMtr(void);

// R backwards L forwards
char Toast_ZeroPt_CW(char rate);

// L backwards R forwards
char Toast_ZeroPt_CCW(char rate);

// Runs R motor backwards
char Toast_Pivot_BCW(int deg, char rate);

// Runs L motor backwards
char Toast_Pivot_BCCW(int deg, char rate);

// Runs R motor backwards
char Toast_Pivot_FCW(int deg, char rate);

// Runs L motor backwards
char Toast_Pivot_FCCW(int deg, char rate);


// Runs R motor backwards
char Toast_P_BCW(int deg, char rate);

// Runs L motor backwards
char Toast_P_BCCW(int deg, char rate);

// Runs R motor backwards
char Toast_P_FCW(int deg, char rate);

// Runs L motor backwards
char Toast_P_FCCW(int deg, char rate);



int CVT_Speed(char newSpeed);

#endif