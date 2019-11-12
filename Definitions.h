//DEFINITIONS

#pragma systemfile

#define     JOY_DRIVE_V     		vexJSLeftV
#define     JOY_DRIVE_H     		vexJSRightH
#define     JOY_THRESHOLD   		25
#define			IntakeBar_UP				Btn6UXmtr2
#define			IntakeBar_DOWN			Btn6DXmtr2
#define			Goliath_Intake			Btn8DXmtr2
#define			Goliath_Outtake			Btn7DXmtr2
#define			Lift_UP							Btn6UXmtr2
#define			Lift_DOWN						Btn6DXmtr2
#define			Mobile_IN						Btn5U
#define 		Mobile_OUT					Btn5D

#define MOTOR_NUM               kNumbOfTotalMotors
#define MOTOR_MAX_VALUE         127
#define MOTOR_MIN_VALUE         (-127)
#define MOTOR_DEFAULT_SLEW_RATE 10      // Default will cause 375mS from full fwd to rev
#define MOTOR_LIFT_SLEW_RATE		40
#define MOTOR_FAST_SLEW_RATE    256     // essentially off
#define MOTOR_TASK_DELAY        20      // task 1/frequency in mS (about 66Hz)
#define MOTOR_DEADBAND          10

#define PID_LSENSOR_INDEX				leftEnc
#define PID_RSENSOR_INDEX				rightEnc
#define PID_ENCODER_SCALE    		1/34.708333 //to scale inches into ticks. there are 34.708333 ticks per inch
#define PID_GYRO_SCALE					1/10				//to scale degrees into ticks. there are 10 ticks per degree for gyro
#define PID_INTEGRAL_LIMIT  		50

#define PID_NUM									3					//So that you can iterate through the different sets in PID Value Struct Array
#define PID_LEFT								0
#define PID_RIGHT								1
#define PID_GYRO								2

#define INTAKE_BAR_UP_POS				5					//pot values for intake bar up and down position
#define INTAKE_BAR_DOWN_POS			1200



typedef struct{
	float	pid_Kp;
	float	pid_Ki;
	float	pid_Kd;
	float	pidSensorCurrentValue;
	float	pidError;
	float	pidLastError;
	float	pidIntegral;
	float	pidDerivative;
	float	pidP;
	float	pidI;
	float	pidD;
	float pidDrive;
} PIDVARS;


//GLOBAL VARIABLES

int TARGETTURN;
int ERROR;


int initMogoVal;

static int   drivePIDRunning = 0;
static int	 turnPIDRunning = 0;
static float distancePidRequestValue;		//in terms of inches
static float turnPidRequestValue;					//in terms of degrees
float	pidDrive;

PIDVARS PIDStruct[PID_NUM];
