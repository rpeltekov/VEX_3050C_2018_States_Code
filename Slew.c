#include "DefinitionsSingleController.h"

int motorReq[ MOTOR_NUM ];// Array to hold requested speed for the motors
int motorSlew[ MOTOR_NUM ];// Array to hold "slew rate" for the motors, the maximum change every time the task runs checking current mootor speed.

task MotorSlewRateTask()
{
	unsigned int motorIndex;
	int motorTmp;
	for(motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++){	// Initialize stuff
		motorReq[motorIndex] = 0;
		if (motorIndex == liftR || motorIndex == liftL || motorIndex == gol){
			motorSlew[motorIndex] = MOTOR_LIFT_SLEW_RATE;
		}
		else
			motorSlew[motorIndex] = MOTOR_DEFAULT_SLEW_RATE;
	}
	while( true ){
		for(motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
		{
			motorTmp = motor[ motorIndex ];

			if(motorTmp != motorReq[motorIndex]){					// Do we need to change the motor value ?
				if(motorReq[motorIndex] > motorTmp){				// increasing motor value
					motorTmp += motorSlew[motorIndex];
					if(motorTmp > motorReq[motorIndex])				// limit
						motorTmp = motorReq[motorIndex];
				}
				if(motorReq[motorIndex] < motorTmp){				// decreasing motor value
					motorTmp -= motorSlew[motorIndex];
					if(motorTmp < motorReq[motorIndex])				// limit
						motorTmp = motorReq[motorIndex];
				}
				motor[motorIndex] = motorTmp;
			}
		}
		wait1Msec( MOTOR_TASK_DELAY );
	}
}
