#include "Slew.c"
//Functions for Various Things

//SLEW REQUEST FUNCTIONS
void DriveLeftMotor_( int value ){
	motorReq[driveLeft_1] = value;
	motorReq[driveLeft_2] = value;
}

void DriveRightMotor_( int value ){
	motorReq[driveRight_1] = value;
	motorReq[driveRight_2] = value;
}

void LiftLeftRequest_(int power){
	motorReq[liftL] =	power;
}
void LiftRightRequest_(int power){
	motorReq[liftR] = power;
}

void IntakeBarRequest_(int power){
	motorReq[intBarL] = power;
	motorReq[intBarR] = -power;
}

void MogoRequest_(int power){
	motorReq[mogo] = power;
}

void GolRequest_(int power){
	motorReq[gol] = power;
}

//

void SensorInit(){
	SensorValue[leftEnc ] = 0;
	SensorValue[rightEnc] = 0;
	SensorValue[gyro] = 0;
	downMogoVal = SensorValue[mogopot];
	downLiftValL = SensorValue[liftLPot];
	downLiftValR = SensorValue[liftRPot];
	upIntakeVal = SensorValue[intBarPot];
}

void BearTurn_(int distance, int degrees){
	distancePidRequestValue = distance;
}

int LiftPower(){
	return(127 * (vexRT[Lift_UP] - vexRT[Lift_DOWN]));
}

int MogoPower(){
	return(127 *(vexRT[Mobile_IN] - vexRT[Mobile_OUT]));
}

int IntakeBarPower(){
	if (SensorValue[intBarPot] <= INTAKE_BAR_UP_POS || SensorValue[intBarPot] >= 2500)
		return(-127 * vexRT[IntakeBar_DOWN]);
	else if (SensorValue[intBarPot] >= INTAKE_BAR_DOWN_POS)
		return(127 * vexRT[IntakeBar_UP]);
	else
		return(127 * (vexRT[IntakeBar_UP] - vexRT[IntakeBar_DOWN]));
}

int GolPower(){
	if(SensorValue[intBarPot] <=10){
		return(MOTOR_MAX_VALUE);
	}else if(vexRT[Goliath_Intake] == 1 && vexRT[Goliath_Outtake] == 0){
		return(MOTOR_MIN_VALUE);
	}else if(vexRT[Goliath_Outtake] == 1 && vexRT[Goliath_Intake] == 0){
		return(MOTOR_MAX_VALUE);
	}
	else
		return (-15);
}

string mainBattery, backupBattery;
void LCD_Battery(){
		clearLCDLine(0);											// Clear line 1 (0) of the LCD
		clearLCDLine(1);											// Clear line 2 (1) of the LCD

		//Display the Primary Robot battery voltage
		displayLCDString(0, 0, "Primary: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
		displayNextLCDString(mainBattery);

		//Display the Backup battery voltage
		displayLCDString(1, 0, "Backup: ");
		sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');	//Build the value to be displayed
		displayNextLCDString(backupBattery);

		//Short delay for the LCD refresh rate
		wait1Msec(100);
}

void CLEARWARNING(){ //DO NOT RUN:::::JUST FOR CLEARING RANDOM ERRORS AT THE BOTTOM// AND HOLDING CODE tHAT IS NOT BEING CURRENTLY USED
	//bool btn8dNotPressed = true;
	//bool btn7dNotPressed = true;
	//int	turnOffCounter = 0;
	BearTurn_(0,0);
	CLEARWARNING();
}
