#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx.h"

typedef enum
{
	ROBOT_STOP = 0,
	ROBOT_FORWARD,
	ROBOT_BACKWARD,
	ROBOT_LEFT,
	ROBOT_RIGHT,
} RobotState_t;

void Motor_Init(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);

#endif
