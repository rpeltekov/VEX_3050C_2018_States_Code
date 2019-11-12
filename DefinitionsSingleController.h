//DEFINITIONS

#pragma systemfile

#define     JOY_DRIVE_V     		vexJSLeftV
#define     JOY_DRIVE_H     		vexJSRightH
#define     JOY_THRESHOLD   		25
#define			IntakeBar_UP				Btn5UXmtr2
#define			IntakeBar_DOWN			Btn5DXmtr2
#define			Goliath_Intake			Btn8DXmtr2
#define			Goliath_Outtake			Btn7DXmtr2
#define			Lift_UP							Btn6UXmtr2
#define			Lift_DOWN						Btn6DXmtr2
#define			Mobile_IN						Btn6D
#define 		Mobile_OUT					Btn6U

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
#define PID_LIFT_POT_SCALE			1/50				//IDK THIS YET//////////// /////////// ///////////////////// /////////// ////////// ///////// //////
#define PID_INTEGRAL_LIMIT  		50

#define PID_NUM									5					//So that you can iterate through the different sets in PID Value Struct Array
#define PID_LEFT								0
#define PID_RIGHT								1
#define PID_GYRO								2
#define PID_LIFT_RIGHT					3
#define PID_LIFT_LEFT						4

#define INTAKE_BAR_UP_POS				5					//pot values for intake bar up and down position
#define INTAKE_BAR_DOWN_POS			1200

#define MOGO_IN_POS							300
#define MOGO_OUT_POS						1800

#define LIFT_DOWN_POSITION			3500
#define	LIFT_UP_POSITION				1500

#define MAX_CONES								17

#define HEIGHT_CONE_ADDED				2.75			//(in) height added by every cone on the stack
#define ARM_LENGTH							15				//(in) length of arm in
#define MID_SECTION_LENGTH			10.5			//(in) length of the middle section
#define HEIGHT_JOINT						3					//(in) height of joint above mobile goal

#define HEIGHT_DRIVER_LOADER		19				//(in) Height at which robot needs to move for driver loads in comparison to base of lift tower
#define HEIGHT_CHINA_NO_CONE		36				//(in) Height of the China without cone in comparison to base of lift tower

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

int downMogoVal;
int downLiftValL;
int downLiftValR;
int upIntakeVal;

static int		drivePIDRunning = 0;
static int		turnPIDRunning = 0;
static int		liftPIDRunning = 0;
static float	distancePidRequestValue;		//in terms of inches
static float	turnPidRequestValue;				//in terms of degrees
static float	liftPidRequestValue;				//in terms of degrees for lift arm
float					pidDrive;

PIDVARS PIDStruct[PID_NUM];

float coneAngle[MAX_CONES + 1];
