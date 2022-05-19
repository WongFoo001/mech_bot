#ifndef PINOUT_DEFINITIONS_H
#define	PINOUT_DEFINITIONS_H

#include "xc.h"
#include "BOARD.h"
#include "pwm.h"
#include "AD.h"

/**********************************
 * MOTORS
 **********************************/
// Motor Control PWM Output Signals
#define RIGHT_MTR_PWM PWM_PORTY12 // Y-12
#define LEFT_MTR_PWM  PWM_PORTY10 // Y-10

// Motor H-Bridge Directions
#define RIGHT_MTR_DIR_TRIS TRISDbits.TRISD4 // X-11
#define RIGHT_MTR_INV_TRIS TRISFbits.TRISF6 // X-8
#define LEFT_MTR_DIR_TRIS  TRISGbits.TRISG7 // X-7
#define LEFT_MTR_INV_TRIS  TRISGbits.TRISG6 // X-5

// Motor H-Bridge Outputs
#define RIGHT_MTR_DIR LATDbits.LATD4 // X-11
#define RIGHT_MTR_INV LATFbits.LATF6 // X-8
#define LEFT_MTR_DIR  LATGbits.LATG7 // X-7
#define LEFT_MTR_INV  LATGbits.LATG6 // X-5

/**********************************
 * BUMPERS
 **********************************/
// Front Bumper Ports
#define FRONT_LEFT_BUMPER        PORTEbits.RE3 // Z-5
#define FRONT_CENTER_BUMPER      PORTFbits.RF1 // Z-4
#define FRONT_RIGHT_BUMPER       PORTEbits.RE4 // Z-3
// Rear Bumper Ports
#define REAR_LEFT_BUMPER         PORTEbits.RE1 // Z-9
#define REAR_CENTER_BUMPER       PORTDbits.RD8 // Z-8
#define REAR_RIGHT_BUMPER        PORTEbits.RE2 // Z-7

// Front Bumper Directions
#define FRONT_LEFT_BUMPER_TRIS   TRISEbits.TRISE3 // Z-5
#define FRONT_CENTER_BUMPER_TRIS TRISFbits.TRISF1 // Z-4
#define FRONT_RIGHT_BUMPER_TRIS  TRISEbits.TRISE4 // Z-3
// Rear Bumper Directions
#define REAR_LEFT_BUMPER_TRIS    TRISEbits.TRISE1 // Z-9
#define REAR_CENTER_BUMPER_TRIS  TRISDbits.TRISD8 // Z-8
#define REAR_RIGHT_BUMPER_TRIS   TRISEbits.TRISE2 // Z-7

/**********************************
 * FLOOR TAPE SENSORS
 **********************************/
// Floor Tape Sensor Analog Inputs
#define AFT_L  AD_PORTV5 // V-5
#define AFT_R  AD_PORTV6 // V-6
#define FORE_L AD_PORTV4 // V-4
#define FORE_R AD_PORTV3 // V-3

// Floor Tape Sensor Enable Direction
#define TAPE_SENSOR_DIR TRISBbits.TRISB8 // V-7

// Floor Tape Sensor Enable Output
#define TAPE_SENSOR_EN LATBbits.LATB8 // V-7

/**********************************
 * FACE TAPE SENSOR
 **********************************/
// Face Tape Sensor Analog Input
#define FACE_SENSOR AD_PORTV8 // V-8

// Face Tape Sensor Enable Direction
#define FACE_SENSOR_DIR TRISDbits.TRISD3 // Y-4

// Face Tape Sensor Enable Output
#define FACE_SENSOR_EN LATDbits.LATD3 // Y-4

/**********************************
 * BALL RAMP SERVO
 **********************************/
// Ball Ramp Servo Output
#define RAMP_SERVO_PIN RC_PORTY06 // Y-6

/**********************************
 * BALL LOADER PWM
 **********************************/
// Ball Loader PWM Output
#define BALL_PWM PWM_PORTZ06 // Z-6

/**********************************
 * TRACKWIRES
 **********************************/
// Trackwire Analog Inputs
#define FORE_WIRE AD_PORTW3 // W-3
#define AFT_WIRE AD_PORTW4 // W-4

/**********************************
 * BEACON DETECTOR
 **********************************/
// Beacon Detector Analog Input
#define BEACON_IN AD_PORTW5 // W-5

/**********************************
 * HALL EFFECT SENSOR
 **********************************/
// Hall Effect Sensor Direction
#define HALL_SENSOR_DIR TRISEbits.TRISE0 // Z-11

// Hall Effect Sensor Signal Port
#define HALL_SENSOR_SIG PORTEbits.RE0 // Z-11

/**********************************
 * UART Enable
 **********************************/
// UART Enable Direction
#define UART_EN_TRIS TRISBbits.TRISB0 // X-4

// UART Output Signal
#define UART_EN LATBbits.LATB0 // X-4

#endif	/* PINOUT_DEFINITIONS_H */

