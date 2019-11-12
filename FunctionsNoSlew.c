#include "DefinitionsSingleController.h"
//Functions for Various Things

//SLEW REQUEST FUNCTIONS
void DriveLeftMotor_( int value ){
	motor[driveLeft_1] = value;
	motor[driveLeft_2] = value;
}

void DriveRightMotor_( int value ){
	motor[driveRight_1] = value;
	motor[driveRight_2] = value;
}

void LiftLeftRequest_(int power){
	motor[liftL] =	power;
}
void LiftRightRequest_(int power){
	motor[liftR] = power;
}

void IntakeBarRequest_(int power){
	motor[intBarL] = power;
	motor[intBarR] = -power;
}

void MogoRequest_(int power){
	motor[mogo] = power;
}

void GolRequest_(int power){
	motor[gol] = power;
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

void DriveSensorInit(){
	SensorValue[leftEnc ] = 0;
	SensorValue[rightEnc] = 0;
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

task golStay(){
	motor[gol] = -10;
}
task golOut(){
	stopTask(golStay);
	motor[gol] = 127;
	wait1Msec(1000);
	motor[gol] = 0;
	stopTask(golOut);
}
task golIn(){
	motor[gol] = -127;
	wait1Msec(900);
	motor[gol] = -15;
	stopTask(golIn);
}
task golInLong(){
	motor[gol] = -127;
	wait1Msec(1500);
	motor[gol] = -10;
	stopTask(golInLong);
}
task liftUP(){
		motor[liftL] = motor[liftR] = 127;
		waitUntil((SensorValue[liftLPot] + SensorValue[liftRPot]) / 2 < 3100);
		motor[liftL] = motor[liftR] = 10;
		stopTask(liftUP);
}
void liftUPhigh(){
		motor[liftL] = motor[liftR] = 127;
		waitUntil((SensorValue[liftLPot] + SensorValue[liftRPot]) / 2 < 2800);
		motor[liftL] = motor[liftR] = 10;
}
task liftGoDown(){
		motor[liftL] = motor[liftR] = -127;
		waitUntil((SensorValue[liftLPot] + SensorValue[liftRPot]) / 2 > 3500);
		motor[liftL] = motor[liftR] = 0;
		stopTask(liftGoDown);
}
task liftGoDownShort(){
		motor[liftL] = motor[liftR] = -127;
		wait1Msec(200);
		motor[liftL] = motor[liftR] = 0;
		stopTask(liftGoDownShort);
}
task liftGoDownCone2(){
		motor[liftL] = motor[liftR] = -127;
		waitUntil((SensorValue[liftLPot] + SensorValue[liftRPot]) / 2 > 3200);
		motor[liftL] = motor[liftR] = 0;
		stopTask(liftGoDownCone2);
}

task mogoOut(){
	motor[mogo] = -127;
	waitUntil(SensorValue[mogopot] >= 4050);
	motor[mogo] = 0;
	stopTask(mogoOut);
}
task mogoIn(){
	motor[mogo] = 127;
	waitUntil(SensorValue[mogopot] <= 420);
	motor[mogo] = 0;
	stopTask(mogoIn);
}
task IntBarDown(){
	IntakeBarRequest_(-80);
	wait1Msec(450);
	IntakeBarRequest_(0);
	stopTask(IntBarDown);
}

task IntBarUp(){
	IntakeBarRequest_(127);
	wait1Msec(300);
	IntakeBarRequest_(0);
	stopTask(IntBarUp);
}

task IntBarUp2(){
	IntakeBarRequest_(127);
	wait1Msec(400);
	IntakeBarRequest_(0);
	stopTask(IntBarUp2);
}
int IntakeBarPower(){
	//int no
	//if (vexRT[Btn7R] == 1 && btn7RnotPressed){

	//}
	//if (SensorValue[intBarPot] <= INTAKE_BAR_UP_POS || SensorValue[intBarPot] >= 00)
	//	return(-127 * vexRT[IntakeBar_DOWN]);
	//else if (SensorValue[intBarPot] >= INTAKE_BAR_DOWN_POS)
	//	return(127 * vexRT[IntakeBar_UP]);
	//else
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
		return (-20);
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
