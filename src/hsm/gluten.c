#include "gluten.h"

int main(void){
    ES_Return_t ErrorType;
    
    BOARD_Init();
    Toaster_Init(); // Initializing motors
    CROISSANT_Init();
    printf("Toast Main Starting\r\n");
    
    // hardware initialization 
    //    Tape_Init();
    Bumpers_Init();
    AD_Init();
    RC_Init(); // Servo init for RC ramp
    RC_AddPins(RAMP_SERVO_PIN); 
    RC_SetPulseTime(RAMP_SERVO_PIN, 1950);  // retract ramp
    
    PWM_AddPins(BALL_PWM);
    PWM_SetDutyCycle(BALL_PWM, 0);
    
    //         Beac Detec   FR Tape   FL Tape  AL Tape  AR Tape
    AD_AddPins(BEACON_IN | FORE_R | FORE_L | AFT_L | AFT_R);
    AD_AddPins(FORE_WIRE | AFT_WIRE | FACE_SENSOR);
    
    // Configure Hall Effect sensor direction
    HALL_SENSOR_DIR = 1; // Config hall as input
    
    // init es framework
    ErrorType = ES_Initialize();
    if(ErrorType == Success){
        ErrorType = ES_Run();
    }
    
    switch(ErrorType){
        case FailedPointer:
            printf("Failed NULL pointer");
            break;
        case FailedInit:
            printf("Failed Initialization");
            break;
        default:
            printf("Other failure: %d", (int)ErrorType);
            break;
    }
    for(;;)
        ;
};
