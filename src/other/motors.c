#include "motors.h"

#define WHEEL_TO_WHEEL 203.0 // mm
#define WHEEL_CIRC 282.75 // mm    pi*90mm
#define RPM_V 10.0 // rpm/v          12v -> 120 rmp
#define MIN_TO_MIL 60000 // ms
#define NOM_BAT_VOLT 9.9

#define OUTPUT 0
#define INPUT  1

#define FORWARD  1
#define BACKWARD 0
#define TOAST_MAX_SPEED 100
#define TOAST_MIN_SPEED 60
#define MAX_PWM 1000

#define RIGHT_MTR_PWM PWM_PORTY12
#define LEFT_MTR_PWM PWM_PORTY10  

// Signal Directions
#define RIGHT_MTR_DIR_TRIS TRISDbits.TRISD4
#define RIGHT_MTR_INV_TRIS TRISFbits.TRISF6
#define LEFT_MTR_DIR_TRIS  TRISGbits.TRISG7
#define LEFT_MTR_INV_TRIS  TRISGbits.TRISG6

// Port References
#define RIGHT_MTR_DIR LATDbits.LATD4 // X 11
#define RIGHT_MTR_INV LATFbits.LATF6 // X 8
#define LEFT_MTR_DIR  LATGbits.LATG7 // X 7
#define LEFT_MTR_INV  LATGbits.LATG6 // X 5

#define INERTIAL_OFFSET 200
#define PARALLEL_OFFSET 100
#define BOOSTER 10
#define RAT 4
//A dir x 8, 11
//B dir x 7, 9
//A pwm y 10
//B pwm y 12

/*******************************************************************************
 * PUBLIC FUNCTION                                                *
 ******************************************************************************/
// returns the time necessary to turn some number of deg at the rate 
int deg_time(int deg, char rate, int off_set){
    //printf("float rate %f\r\n", (float)rate);
    float arc_len = (deg/360.0) * (3.14*(WHEEL_TO_WHEEL/2)); // mm
    //printf("arc len %f\r\n", arc_len);
    float rot = arc_len / WHEEL_CIRC; // r  rotations of the wheel
    //printf("rot %f\r\n", rot);
    float v = RPM_V * NOM_BAT_VOLT * ((float)rate/100); // rpm    how many rpm for voltage and dc
    //printf("v %f\r\n", v);
    float t = rot / (v / MIN_TO_MIL);
    //printf("PIVOT TIME FLOAT: %f\r\n");
    return (int)t + off_set;
    
}

void Toaster_Init(void){
    // Initialize Motor PWM Signals
    PWM_Init();
    PWM_SetFrequency(500);
    PWM_AddPins(RIGHT_MTR_PWM | LEFT_MTR_PWM);
    
    // Motor signal direction initializations
    RIGHT_MTR_DIR_TRIS = OUTPUT;
    RIGHT_MTR_INV_TRIS = OUTPUT;
    LEFT_MTR_DIR_TRIS  = OUTPUT;
    LEFT_MTR_INV_TRIS  = OUTPUT;
    
    // Motor signal value initializations
    RIGHT_MTR_DIR = FORWARD;
    RIGHT_MTR_INV = ~RIGHT_MTR_DIR;
    LEFT_MTR_DIR  = FORWARD;
    LEFT_MTR_INV  = ~LEFT_MTR_DIR;
}

int CVT_Speed(char newSpeed){
    int remap = ((((TOAST_MAX_SPEED - TOAST_MIN_SPEED) * (int)newSpeed) / TOAST_MAX_SPEED) + TOAST_MIN_SPEED) * 10;
    return remap;
}

char Toast_LeftMtrSpeed(char newSpeed){
    if ((newSpeed < -TOAST_MAX_SPEED) || (newSpeed > TOAST_MAX_SPEED)) {
        return (ERROR);
    }
    if (newSpeed < 0) {
        LEFT_MTR_DIR = BACKWARD;
        LEFT_MTR_INV = FORWARD;
        newSpeed = newSpeed * (-1); // set speed to absolute value
    } else {
        LEFT_MTR_DIR = FORWARD;
        LEFT_MTR_INV = BACKWARD;
    }
    
//    char ret = PWM_SetDutyCycle(LEFT_MTR_PWM, CVT_Speed(newSpeed));
//    if (ret == ERROR) {
//        printf("ERROR: setting channel LEFT speed!\n");
//        return (ERROR);
//    }
//    return (SUCCESS);
    if (PWM_SetDutyCycle(LEFT_MTR_PWM, newSpeed * (MAX_PWM / TOAST_MAX_SPEED)) == ERROR) {
        //puts("\aERROR: setting channel 1 speed!\n");
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
        RIGHT_MTR_INV = FORWARD;
        newSpeed = newSpeed * (-1); // set speed to absolute value
    } else {
        RIGHT_MTR_DIR = FORWARD;
        RIGHT_MTR_INV = BACKWARD;
    }
    
//    char ret = PWM_SetDutyCycle(RIGHT_MTR_PWM, CVT_Speed(newSpeed));
//    if (ret == ERROR) {
//        printf("ERROR: setting channel RIGHT speed!\r\n");
//        return (ERROR);
//    }
//    return (SUCCESS);
    if (PWM_SetDutyCycle(RIGHT_MTR_PWM, newSpeed * (MAX_PWM / TOAST_MAX_SPEED)) == ERROR) {
        //puts("\aERROR: setting channel 1 speed!\n");
        return (ERROR);
    }
    return (SUCCESS);
}


// set speed of both motors
char Toast_BothMtrSpeed(char newSpeed){
    char l_ret;
    char r_ret;
    l_ret = Toast_LeftMtrSpeed(newSpeed);
    r_ret = Toast_RightMtrSpeed(newSpeed);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_BothMtrSpeed %d  %d \r\n", l_ret, r_ret);
        return ERROR;
    }
    return SUCCESS;
}

char Toast_StopMtr(void){
    PWM_SetDutyCycle(RIGHT_MTR_PWM, 0);
    PWM_SetDutyCycle(LEFT_MTR_PWM, 0);
    return SUCCESS;
}

// R backwards L forwards
char Toast_ZeroPt_CW(char rate){
    char l_ret;
    char r_ret;
    l_ret = Toast_LeftMtrSpeed(rate);
    r_ret = Toast_RightMtrSpeed(-rate);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_ZeroPt_CW");
        return ERROR;
    }
    return SUCCESS;
    
}


// L backswards R forwards
char Toast_ZeroPt_CCW(char rate){
    char l_ret;
    char r_ret;
    l_ret = Toast_LeftMtrSpeed(-rate);
    r_ret = Toast_RightMtrSpeed(rate);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_ZeroPt_CCW");
        return ERROR;
    }
    return SUCCESS;
}


// Runs R motor backwards
char Toast_Pivot_BCW(int deg, char rate){
    Toast_StopMtr();
    char l_ret;
    char r_ret;
    int time = deg_time(deg, rate, INERTIAL_OFFSET);
    //printf("PIVOT TIME: %d\r\n", time);
    l_ret = Toast_LeftMtrSpeed(-BOOSTER);
    r_ret = Toast_RightMtrSpeed(-rate);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_Pivot_CW");
        return ERROR;
    }
    ES_Timer_InitTimer(MANEUVER_TIMER, time); // know when to stop turning
    return SUCCESS;
}


// Runs L motor backwards
char Toast_Pivot_BCCW(int deg, char rate){
    char l_ret;
    char r_ret;
    int time = deg_time(deg, rate, INERTIAL_OFFSET);
    l_ret = Toast_LeftMtrSpeed(-rate);
    r_ret = Toast_RightMtrSpeed(-BOOSTER);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_Pivot_CCW");
        return ERROR;
    }
    ES_Timer_InitTimer(MANEUVER_TIMER, time); // know when to stop turning
    
    return SUCCESS;
}


// Runs L motor FORWARDS
char Toast_Pivot_FCW(int deg, char rate){
    Toast_StopMtr();
    char l_ret;
    char r_ret;
    int time = deg_time(deg, rate, INERTIAL_OFFSET);
    //printf("PIVOT TIME: %d\r\n", time);
    l_ret = Toast_LeftMtrSpeed(rate);
    r_ret = Toast_RightMtrSpeed(BOOSTER);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_Pivot_CW");
        return ERROR;
    }
    ES_Timer_InitTimer(MANEUVER_TIMER, time); // know when to stop turning
    return SUCCESS;
}


// Runs R motor FORWARDS
char Toast_Pivot_FCCW(int deg, char rate){
    char l_ret;
    char r_ret;
    int time = deg_time(deg, rate, INERTIAL_OFFSET);
    l_ret = Toast_LeftMtrSpeed(BOOSTER);
    r_ret = Toast_RightMtrSpeed(rate);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_Pivot_CCW");
        return ERROR;
    }
    ES_Timer_InitTimer(MANEUVER_TIMER, time); // know when to stop turning
    
    return SUCCESS;
}








// Runs R motor backwards used for parallelize
char Toast_P_BCW(int deg, char rate){
    Toast_StopMtr();
    char l_ret;
    char r_ret;
    int time = deg_time(deg, rate, PARALLEL_OFFSET);
    //printf("PIVOT TIME: %d\r\n", time);
    l_ret = Toast_LeftMtrSpeed(-rate/RAT);
    r_ret = Toast_RightMtrSpeed(-rate);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_Pivot_CW");
        return ERROR;
    }
    ES_Timer_InitTimer(MANEUVER_TIMER, time); // know when to stop turning
    return SUCCESS;
}


// Runs L motor backwards used for parallelize
char Toast_P_BCCW(int deg, char rate){
    char l_ret;
    char r_ret;
    int time = deg_time(deg, rate, PARALLEL_OFFSET);
    l_ret = Toast_LeftMtrSpeed(-rate);
    r_ret = Toast_RightMtrSpeed(-rate/RAT);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_Pivot_CCW");
        return ERROR;
    }
    ES_Timer_InitTimer(MANEUVER_TIMER, time); // know when to stop turning
    
    return SUCCESS;
}


// Runs L motor FORWARDS used for parallelize
char Toast_P_FCW(int deg, char rate){
    Toast_StopMtr();
    char l_ret;
    char r_ret;
    int time = deg_time(deg, rate, PARALLEL_OFFSET);
    //printf("PIVOT TIME: %d\r\n", time);
    l_ret = Toast_LeftMtrSpeed(rate);
    r_ret = Toast_RightMtrSpeed(rate/RAT);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_Pivot_CW");
        return ERROR;
    }
    ES_Timer_InitTimer(MANEUVER_TIMER, time); // know when to stop turning
    return SUCCESS;
}


// Runs R motor FORWARDS used for parallelize
char Toast_P_FCCW(int deg, char rate){
    char l_ret;
    char r_ret;
    int time = deg_time(deg, rate, PARALLEL_OFFSET);
    l_ret = Toast_LeftMtrSpeed(rate/RAT);
    r_ret = Toast_RightMtrSpeed(rate);
    if(l_ret == ERROR || r_ret == ERROR){
        l_ret = Toast_LeftMtrSpeed(0);
        r_ret = Toast_RightMtrSpeed(0);
        printf("ERROR in Toast_Pivot_CCW");
        return ERROR;
    }
    ES_Timer_InitTimer(MANEUVER_TIMER, time); // know when to stop turning
    
    return SUCCESS;
}


char Toast_Halt(){
    
    char l_ret;
    char r_ret;
    l_ret = Toast_LeftMtrSpeed(0);
    r_ret = Toast_RightMtrSpeed(0);
    if(l_ret == ERROR || r_ret == ERROR){
        printf("ERROR in Toast_BothMtrSpeed");
        return ERROR;
    }
    return SUCCESS;
}
