/* 
 * File:   toaster.h
 * Author: juwewong
 *
 * Created on November 16, 2021, 2:04 PM
 */
/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/
/**
 * @Function Toaster_Init(void)
 * @param None
 * @return None
 * @brief  Performs all the initialization necessary for the toaster. this includes initializing
 *         the PWM module, sensors, data pins
 * @author Justin Wong, 2021/16/11 */
void Toaster_Init(void);

/**
 * @Function Toast_LeftMtrSpeed(char newSpeed)
 * @param newSpeed - A value between -100 and 100 which is the new speed
 *        of the motor. 0 stops the motor. A negative value is reverse.
 * @return SUCCESS or ERROR
 * @brief  This function is used to set the speed and direction of the left motor.
 * @author Justin Wong, 2021/16/11 */
char Toast_LeftMtrSpeed(char newSpeed);

/**
 * @Function Toast_RightMtrSpeed(char newSpeed)
 * @param newSpeed - A value between -100 and 100 which is the new speed
 *        of the motor. 0 stops the motor. A negative value is reverse.
 * @return SUCCESS or ERROR
 * @brief  This function is used to set the speed and direction of the right motor.
 * @author Justin Wong, 2021/16/11 */
char Toast_RightMtrSpeed(char newSpeed);
