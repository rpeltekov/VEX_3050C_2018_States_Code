#pragma config(Sensor, in1,    mogopot,        sensorPotentiometer)
#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Sensor, in3,    rightLine,      sensorLineFollower)
#pragma config(Sensor, in4,    leftLine,       sensorLineFollower)
#pragma config(Sensor, in5,    intBarPot,      sensorPotentiometer)
#pragma config(Sensor, in6,    liftLPot,       sensorPotentiometer)
#pragma config(Sensor, in7,    liftRPot,       sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  liftDown,       sensorTouch)
#pragma config(Sensor, dgtl4,  mogoSonar,      sensorSONAR_cm)
#pragma config(Sensor, dgtl11, rightEnc,       sensorQuadEncoder)
#pragma config(Motor,  port1,           driveRight_2,  tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port2,           driveLeft_1,   tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port3,           liftL,         tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port4,           intBarL,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           gol,           tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port6,           intBarR,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port7,           liftR,         tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           driveRight_1,  tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           mogo,          tmotorNone, openLoop)
#pragma config(Motor,  port10,          driveLeft_2,   tmotorServoContinuousRotation, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

void pre_auton(){
	bLCDBacklight = true;									// Turn on LCD Backlight


	//ConeAngleInit();
	SensorInit();
}

task autonomous(){
	SensorInit();
	PIDInit();
	//startTask(MotorSlewRateTask);
	//PointTurn_(90);
	stationary();
}

task usercontrol()
{
	//startTask( MotorSlewRateTask );
	//for(unsigned long motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)	// Initialize stuff
	//	motorReq[motorIndex] = 0;
	int	ctl_v;
	int	ctl_h;
	int	drive_l;
	int	drive_r;

	while( true )
	{
		LCD_Battery(); //for programming to see voltage of battery updated to promote repeatability

		ctl_v = (abs(vexRT[JOY_DRIVE_V]) < 25 ? 0 : vexRT[JOY_DRIVE_V]);
		ctl_h = (abs(vexRT[JOY_DRIVE_H]) < 25 ? 0 : vexRT[JOY_DRIVE_H]);

		drive_l = (ctl_v + ctl_h);
		drive_r = (-ctl_v + ctl_h);

		//drive_l = vexRT[Ch3Xmtr2];
		//drive_r = -vexRT[Ch2Xmtr2];

		DriveLeftMotor_(drive_l);
		DriveRightMotor_(drive_r);
		LiftLeftRequest_(LiftPower());
		LiftRightRequest_(LiftPower());
		IntakeBarRequest_(IntakeBarPower());
		MogoRequest_(MogoPower());
		GolRequest_(GolPower());

		if (vexRT[Btn8d]){
			motor[driveLeft_1] = 15;
			motor[driveLeft_2] = -15;
			motor[driveRight_1] = -15;
			motor[driveRight_2] = 15;
		}

		wait1Msec(20);
	}
}
