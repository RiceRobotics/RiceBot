/*
 * RiceBot.h
 *
 *  Created on: Aug 29, 2014
 *      Author: Keiko
 */

#ifndef RICEBOT_H_
#define RICEBOT_H_

#include "main.h"

//Constants
#define MATH_PI 3.14159265358979323846

//Teams
#define RED -1
#define BLUE 1

//Control Style
#define CTTANKDRIVE  	0
#define CTARCADEDRIVE 	1
#define CTCHEEZYDRIVE  	2
#define CTMECANUMDRIVE  3
#define CTHDRIVE  		4

//Defines how the analog joystick inputs translate to motor outputs (CT-)
int controlStyle;

//Drivetrain Styles
#define DTFOURWHEELS 	0
#define DTSIXWHEELS 	1
#define DTEIGHTWHEELS 	2
#define DTMECANUM 		3
#define DTHOLONOMIC 	4
#define DTSWERVE 		6

//Defines the drivetrain installed on the robot (DT-)
int driveTrainStyle;

//Autonomous Instructions
#define AUTODRIVEBASIC	0
#define AUTODRIVEGYRO 	1
#define AUTOTURNBASIC	2
#define AUTOTURNPID		3
#define AUTOARMPID		4
#define AUTOCOLLECTORS	5
#define AUTODRIVETIME	6


/*
 * The Motor type serves as a wrapper to keep track of all data for each motor on the robot.
 *
 * @param port The port on the Cortex which the motor is plugged into
 * @param out The power output to the motor, between -127 and 127
 * @param reflected If the output to the motor should be reversed. -1 or 1
 */
typedef struct motorStruct {
	unsigned char port;
	int out;
	int reflected;
} Motor;

/*
 * The Pid type contains all data for any individual pid loop we may wish to run.
 *
 * @param running 1 if the Pid instance should be processed/set to motors, else 0
 * @param setPoint The target value for the loop
 * @param *current A pointer to relevant sensor value (CANNOT BE AN ARRAY)
 * @param error The difference between setPoint and &current
 * @param lastError The previous error value, used for derivative calculations
 * @param integral A running sum of all previous errors
 * @param derivative The difference between lastError and error
 * @param kP The coefficient for the proportional term
 * @param kI The coefficient for the integral term
 * @param kD The coefficient for the derivative term
 * @param output The value to be set to the motors
 */
typedef struct pidStruct {
	int running;
	int setPoint;
	int current;
	float error;
	float lastError;
	long integral;
	float derivative;
	float kP;
	float kI;
	float kD;
	int output;
} Pid;

/*
 * The Ricencoder contains data for either an IME or a quadrature encoder
 *
 * @param rawValue The true value of the encoder
 * @param ticksPerRev The number of ticks per revolution of the encoder
 * 						627.2 for the 393 IME in high torque mode (factory default)
 * 						392 for the 393 IME in high speed mode
 * 						360 for the Quadrature Encoder
 * @param mult A multiplier to use as compensation for gear ratio
 * @param adjustedValue The multiplied value of the encoder
 * @param isIME 1 if IME, 0 if quad encoder
 * @param imeAddress The address if IME
 * @param enc (If not IME) The PROS base encoder type needed for retrieving the value
 * @param portTop (If not IME) The port on the Cortex which the top wire of the encoder is plugged into
 * @param portBot (If not IME) The port on the Cortex which the bottom wire of the encoder is plugged into
 * @param reverse (If not IME) Whether the QuadEncoder should count in the opposite direction
 */
typedef struct RicencoderStruct {
	int rawValue;
	float ticksPerRev;
	int mult;
	int adjustedValue;
	int isIME;
	unsigned char imeAddress;
	Encoder enc;
	unsigned char portTop;
	unsigned char portBot;
	bool reverse;
} Ricencoder;

/*
 * The Ricepot is a wrapper for potentiometer use
 *
 * @param port The port on the Cortex which the potentiometer is plugged into
 * @param value The current value of the potentiometer
 */
typedef struct RicepotStruct {
	unsigned char port;
	int value;
} Ricepot;

/*
 * The Ricegyro is a wrapper for gyro use.
 *
 * @param g The PROS Gyro base type
 * @param port The port on the Cortex which the potentiometer is plugged into
 * @param value The current value of the gyro
 * @param multiplier Sensitivity calibration for the gyro. Use 0 for the default value of 196
 */
typedef struct RicegyroStruct {
	Gyro g;
	unsigned char port;
	int value;
	unsigned short multiplier;
} Ricegyro;

//Declaration of all possible Drivetrain motors
Motor MOTDTFrontRight;
Motor MOTDTFrontMidRight;
Motor MOTDTMidRight;
Motor MOTDTBackRight;
Motor MOTDTFrontLeft;
Motor MOTDTFrontMidLeft;
Motor MOTDTMidLeft;
Motor MOTDTBackLeft;

//Declaration of all possible Arm motors
//This is just to make the code more understandable when comparing an arm motor
//name to the physical motor on the robot
Motor MOTARMFront;
Motor MOTARMBack;
Motor MOTARMTop;
Motor MOTARMMiddle;
Motor MOTARMBottom;
Motor MOTARMLeft;
Motor MOTARMRight;
Motor MOTARMTopRight;
Motor MOTARMBottomRight;
Motor MOTARMTopLeft;
Motor MOTARMBottomLeft;

//Declaration of all possible Collector motors
Motor MOTCOL;
Motor MOTCOLLeft;
Motor MOTCOLRight;

unsigned char IMEDTLEFT;
unsigned char IMEDTRIGHT;
unsigned char IMEARMLEFT;
unsigned char IMEARMRIGHT;

//Encoder ENCDTLeft;
//Encoder ENCDTRight;
//Encoder ENCARMLeft;
//Encoder ENCARMRight;

Ricencoder EncDTLeft;
Ricencoder EncDTRight;
Ricencoder EncARMLeft;
Ricencoder EncARMRight;

Ricepot PotARMFront;
Ricepot PotARMLeft;
Ricepot PotARMRight;

Ricegyro gyro;

Pid PidDTLeft;
Pid PidDTRight;
Pid PidARMLeft;
Pid PidARMRight;
Pid PidARMFront;

Motor initMotor(unsigned char port, int reflected);

Pid initPid(float kP, float kI, float kD);

Ricencoder initRicencoder(float ticksPerRev, int mult, int isIME, unsigned char imeAddress,
		unsigned char portTop, unsigned char portBot, bool reverse);

Ricepot initRicepot(unsigned char port);

Ricegyro initRicegyro(unsigned char port, unsigned short multiplier);

void riceBotInitializeIO();

void riceBotInitialize();

void getJoystickForDriveTrain();

void setDriveTrainMotors();

void updateRicencoder(Ricencoder *rc);

void updateRicegyro(Ricegyro *rg);

void autonomousTask(int instruction, int distance, int pow, long timeout);

void processPid(Pid *pidLoop, int current);

int speedRegulator(int speed);

void startIOTask(void *ignore);

void startPidTask(void *ignore);

int max(int a, int b);

int min(int a, int b);

#endif /* RICEBOT_H_ */
