#include "FunctionsNoSlew.c"

///////////////////////////////PIDTASKS AND CALCULATION FUCNTION/////////////////////////////////////////////////////////////////////////////////////////////////
void PIDInit(){
	int pidIndex;

	PIDStruct[PID_LEFT].pid_Kp = 8;						// DriveBase PID Values
	PIDStruct[PID_LEFT].pid_Ki = 0;
	PIDStruct[PID_LEFT].pid_Kd = 55;
	PIDStruct[PID_RIGHT].pid_Kp = 9;
	PIDStruct[PID_RIGHT].pid_Ki = 0;
	PIDStruct[PID_RIGHT].pid_Kd = 50;

	PIDStruct[PID_GYRO].pid_Kp = 2;					//Gyro PID Values OLD ::: 1.8 0 25
	PIDStruct[PID_GYRO].pid_Ki = 0;
	PIDStruct[PID_GYRO].pid_Kd = 25;

	PIDStruct[PID_LIFT_LEFT].pid_Kp = 0;			//Lift PID Values
	PIDStruct[PID_LIFT_LEFT].pid_Ki = 0;
	PIDStruct[PID_LIFT_LEFT].pid_Kd = 0;
	PIDStruct[PID_LIFT_RIGHT].pid_Kp = 0;
	PIDStruct[PID_LIFT_RIGHT].pid_Ki = 0;
	PIDStruct[PID_LIFT_RIGHT].pid_Kd = 0;

	for (pidIndex = 0; pidIndex <= 4; pidIndex ++){
		PIDStruct[pidIndex].pidError			= 0;
		PIDStruct[pidIndex].pidLastError  = 0;	// Init the variables
		PIDStruct[pidIndex].pidIntegral   = 0;
	}
}

int PID_Turn_Calculation(){
	PIDStruct[PID_GYRO].pidSensorCurrentValue = SensorValue[gyro] * PID_GYRO_SCALE;
	PIDStruct[PID_GYRO].pidError = turnPidRequestValue - PIDStruct[PID_GYRO].pidSensorCurrentValue;
	ERROR = PIDStruct[PID_GYRO].pidError;

	if(PIDStruct[PID_GYRO].pid_Ki != 0 ){
		if(abs(PIDStruct[PID_GYRO].pidError) < PID_INTEGRAL_LIMIT )		// If we are inside controlable window then integrate the error
			PIDStruct[PID_GYRO].pidIntegral = PIDStruct[PID_GYRO].pidIntegral + PIDStruct[PID_GYRO].pidError;
		else
			PIDStruct[PID_GYRO].pidIntegral = 0;
	}
	else
		PIDStruct[PID_GYRO].pidIntegral = 0;

	PIDStruct[PID_GYRO].pidDerivative = PIDStruct[PID_GYRO].pidError - PIDStruct[PID_GYRO].pidLastError;
	PIDStruct[PID_GYRO].pidLastError  = PIDStruct[PID_GYRO].pidError;

	PIDStruct[PID_GYRO].pidI = (PIDStruct[PID_GYRO].pid_Ki * PIDStruct[PID_GYRO].pidIntegral);
	PIDStruct[PID_GYRO].pidP = (PIDStruct[PID_GYRO].pid_Kp * PIDStruct[PID_GYRO].pidError);
	PIDStruct[PID_GYRO].pidD = (PIDStruct[PID_GYRO].pid_Kd * PIDStruct[PID_GYRO].pidDerivative);
	return(PIDStruct[PID_GYRO].pidP + PIDStruct[PID_GYRO].pidI + PIDStruct[PID_GYRO].pidD);
}

task PID_Lift(){
	float heightCorrect;
	int 	pidIndex;
	int 	turnOffCounter = 0;

	while(true){
		for (pidIndex = PID_LIFT_RIGHT; pidIndex <= PID_LIFT_LEFT; pidIndex ++){
			if( liftPIDRunning ){

				PIDStruct[pidIndex].pidSensorCurrentValue = ((pidIndex == PID_LEFT ? downLiftValL : downLiftValR) - SensorValue[(pidIndex == PID_LIFT_LEFT ? liftLPot : liftRPot )]) * PID_LIFT_POT_SCALE;
				PIDStruct[pidIndex].pidError = PIDStruct[pidIndex].pidSensorCurrentValue - liftPidRequestValue;

				if(PIDStruct[pidIndex].pid_Ki != 0 ){
					if(abs(PIDStruct[pidIndex].pidError) < PID_INTEGRAL_LIMIT )		// If we are inside controllable window then integrate the error
						PIDStruct[pidIndex].pidIntegral = PIDStruct[pidIndex].pidIntegral + PIDStruct[pidIndex].pidError;
					else
						PIDStruct[pidIndex].pidIntegral = 0;
				}
				else
					PIDStruct[pidIndex].pidIntegral = 0;


				PIDStruct[pidIndex].pidDerivative = PIDStruct[pidIndex].pidError - PIDStruct[pidIndex].pidLastError;
				PIDStruct[pidIndex].pidLastError  = PIDStruct[pidIndex].pidError;

				PIDStruct[pidIndex].pidI = (PIDStruct[pidIndex].pid_Ki * PIDStruct[pidIndex].pidIntegral);
				PIDStruct[pidIndex].pidP = (PIDStruct[pidIndex].pid_Kp * PIDStruct[pidIndex].pidError);
				PIDStruct[pidIndex].pidD = (PIDStruct[pidIndex].pid_Kd * PIDStruct[pidIndex].pidDerivative);
				heightCorrect = PIDStruct[pidIndex].pidP + PIDStruct[pidIndex].pidI + PIDStruct[pidIndex].pidD;

				PIDStruct[pidIndex].pidDrive = (pidIndex == PID_RIGHT ? -heightCorrect : heightCorrect);


				if(PIDStruct[pidIndex].pidDrive > MOTOR_MAX_VALUE)		//Make sure it doesnt go over the limits of robot c
					PIDStruct[pidIndex].pidDrive = MOTOR_MAX_VALUE;
				if(PIDStruct[pidIndex].pidDrive < MOTOR_MIN_VALUE)
					PIDStruct[pidIndex].pidDrive = MOTOR_MIN_VALUE;

				if(pidIndex == PID_LEFT)
					DriveLeftMotor_(-PIDStruct[pidIndex].pidDrive);
				else
					DriveRightMotor_(-PIDStruct[pidIndex].pidDrive);

				if (turnOffCounter <= 50000){
					turnOffCounter ++;
				}
				if(turnOffCounter >= 50000 && PIDStruct[pidIndex].pidDrive <= 4){
					liftPIDRunning = 0;
				}
			}
			else if(!liftPIDRunning){
				//stopTask(PID_Lift);
			}
		}
		wait1Msec(MOTOR_TASK_DELAY - 10);
	}
}

task PID_Drive(){						//WITH STRAIGHT DRIVING CORRECTION
	float gyroCorrect;
	float distanceCorrect;
	int pidIndex;
	int turnOffCounter = 0;

	while(true){
		for (pidIndex = 0; pidIndex <= PID_RIGHT; pidIndex ++){
			if( drivePIDRunning ){

				PIDStruct[pidIndex].pidSensorCurrentValue = (pidIndex == PID_LEFT ? -1 : 1) * SensorValue[(pidIndex == PID_LEFT ? leftEnc : rightEnc )] * PID_ENCODER_SCALE;
				PIDStruct[pidIndex].pidError = PIDStruct[pidIndex].pidSensorCurrentValue - distancePidRequestValue;

				if(PIDStruct[pidIndex].pid_Ki != 0 ){
					if(abs(PIDStruct[pidIndex].pidError) < PID_INTEGRAL_LIMIT )		// If we are inside controllable window then integrate the error
						PIDStruct[pidIndex].pidIntegral = PIDStruct[pidIndex].pidIntegral + PIDStruct[pidIndex].pidError;
					else
						PIDStruct[pidIndex].pidIntegral = 0;
				}
				else
					PIDStruct[pidIndex].pidIntegral = 0;


				PIDStruct[pidIndex].pidDerivative = PIDStruct[pidIndex].pidError - PIDStruct[pidIndex].pidLastError;
				PIDStruct[pidIndex].pidLastError  = PIDStruct[pidIndex].pidError;

				PIDStruct[pidIndex].pidI = (PIDStruct[pidIndex].pid_Ki * PIDStruct[pidIndex].pidIntegral);
				PIDStruct[pidIndex].pidP = (PIDStruct[pidIndex].pid_Kp * PIDStruct[pidIndex].pidError);
				PIDStruct[pidIndex].pidD = (PIDStruct[pidIndex].pid_Kd * PIDStruct[pidIndex].pidDerivative);
				distanceCorrect = PIDStruct[pidIndex].pidP + PIDStruct[pidIndex].pidI + PIDStruct[pidIndex].pidD;

				gyroCorrect = 0;

				PIDStruct[pidIndex].pidDrive = (pidIndex == PID_RIGHT ? -distanceCorrect : distanceCorrect) + (pidIndex == PID_LEFT ? gyroCorrect : -gyroCorrect);


				if(PIDStruct[pidIndex].pidDrive > MOTOR_MAX_VALUE)		//Make sure it doesnt go over the limits of robot c
					PIDStruct[pidIndex].pidDrive = MOTOR_MAX_VALUE;
				if(PIDStruct[pidIndex].pidDrive < MOTOR_MIN_VALUE)
					PIDStruct[pidIndex].pidDrive = MOTOR_MIN_VALUE;

				if(pidIndex == PID_LEFT)
					DriveLeftMotor_(-PIDStruct[pidIndex].pidDrive);
				else
					DriveRightMotor_(-PIDStruct[pidIndex].pidDrive);

				if (turnOffCounter <= 500){
					turnOffCounter ++;
				}
				if(turnOffCounter == 500 && PIDStruct[pidIndex].pidDrive <= 4){
					drivePIDRunning = 0;
				}
			}
			else if(!drivePIDRunning){
				stopTask(PID_Drive);
			}
		}
		wait1Msec(MOTOR_TASK_DELAY - 10);
	}
}

task PID_Turn(){
	float gyroCorrect;
	int pidIndex;
	int turnOffCounter;

	while(true){
		for (pidIndex = 0; pidIndex <= PID_RIGHT; pidIndex ++){
			if( turnPIDRunning ){

				gyroCorrect = PID_Turn_Calculation();

				PIDStruct[pidIndex].pidDrive = (pidIndex == PID_RIGHT ? -gyroCorrect: -gyroCorrect);

				if(PIDStruct[pidIndex].pidDrive > MOTOR_MAX_VALUE)		//Make sure it doesnt go over the limits of robot c
					PIDStruct[pidIndex].pidDrive = MOTOR_MAX_VALUE;
				if(PIDStruct[pidIndex].pidDrive < MOTOR_MIN_VALUE)
					PIDStruct[pidIndex].pidDrive = MOTOR_MIN_VALUE;

				if(pidIndex == PID_LEFT)
					DriveLeftMotor_(-PIDStruct[pidIndex].pidDrive);
				else
					DriveRightMotor_(-PIDStruct[pidIndex].pidDrive);

				if (turnOffCounter <= 500){
					turnOffCounter ++;
				}
				if(turnOffCounter == 500 && PIDStruct[pidIndex].pidDrive <= 4){
					turnPIDRunning = 0;
				}
			}
			else if (!turnPIDRunning){
				stopTask(PID_Turn);
			}
		}
		wait1Msec(MOTOR_TASK_DELAY - 10);
	}
}

////////////////////////////////////////////////////AUTOSTACKING FUCNTIONS//////////////////////////////////////////////////////////////////////////////////////////////////
float NthInternalConeHeight_(int n){
	return(HEIGHT_CONE_ADDED * (float)n);
}

float NthChinaConeHeight_(int n){
	return(HEIGHT_CONE_ADDED * n + HEIGHT_CHINA_NO_CONE);
}


void ConeAngleInit(){
	for(int n = 0; n <= MAX_CONES; n++){
		coneAngle[n] = PI - asin((HEIGHT_CONE_ADDED * n - HEIGHT_JOINT - MID_SECTION_LENGTH)/(2*ARM_LENGTH));
	}
}

float PositionGrab_(int coneN){
	return coneAngle[coneN];
}

task Lift_Task(){
	int coneCount = 0;
	bool btn8Unotpress = true;
	bool btn7Unotpress = true;
	bool btn7Rnotpress = true;
	bool btn8Lnotpress = true;
	while(true){
		if (vexRT[Btn8U] && btn8Unotpress){
			coneCount ++;
			btn8Unotpress = false;
		}
		if (vexRT[Btn7U] && btn7Unotpress){
			coneCount --;
			btn8Unotpress = false;
		}
		if (vexRT[Btn7R] && btn7Rnotpress){
			coneCount = 0;
			btn7Rnotpress = false;
		}

		if (vexRT[Btn8L] && btn8Lnotpress){
			coneCount ++;
		//	LiftLeftRequest_(LiftPower());
		//	LiftRightRequest_(LiftPower());
			btn8Lnotpress = false;
		}

		if (vexRT[Btn8U] == 0)
			btn8Unotpress = true;
		if (vexRT[Btn7U] == 0)
			btn7Unotpress = true;
		if (vexRT[Btn7R] == 0)
			btn7Rnotpress = true;
		if (vexRT[Btn8L] == 0)
			btn8Lnotpress = true;
	}
}

////////////////////////////////////////////////////FUNCTIONS THAT MAKE USE OF PID TASKS AND SUCH///////////////////////////////////////////////////////////////////////////

void DriveForward_(float distance){
	drivePIDRunning = 1;
	startTask(PID_Drive);
	distancePidRequestValue = distance;
}

void PointTurn_(float degrees){
	turnPIDRunning = 1;
	startTask(PID_Turn);
	turnPidRequestValue = degrees;
}




//For Error Removal//
void CLEARWARNING2(){ //DO NOT RUN:::::JUST FOR CLEARING RANDOM ERRORS AT THE BOTTOM// AND HOLDING CODE tHAT IS NOT BEING CURRENTLY USED
	//bool btn8dNotPressed = true;
	//bool btn7dNotPressed = true;
	//int	turnOffCounter = 0;
	if (false){
		DriveForward_(0);
	}
	CLEARWARNING2();
}
