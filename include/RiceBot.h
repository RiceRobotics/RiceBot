/*
 * RiceBot.h
 *
 *  Created on: Aug 29, 2014
 *      Author: Keiko
 */

#ifndef RICEBOT_H_
#define RICEBOT_H_

#include <API.h>

/*
 * The Motor type serves as a wrapper to keep track of all data for each motor on the robot.
 *
 * @param port The motor port on the Cortex which the motor is plugged into
 * @param out The power output to the motor, between -127 and 127
 * @param reflected If the output to the motor should be reversed. -1 or 1
 */
typedef struct motorStruct {
	unsigned char port;
	int out;
	int reflected;
} Ricemotor;

/*
 * The Pid type contains all data for any individual pid loop we may wish to run.
 *
 * @param *sensor A pointer to the sensor's value field
 * @param running 1 if the Pid instance should be processed/set to motors, else 0
 * @param setPoint The target value for the loop
 * @param current The current value of the sensor
 * @param error The difference between setPoint and &current
 * @param lastError The previous error value, used for derivative calculations
 * @param integral A running sum of all previous errors
 * @param derivative The difference between lastError and error
 * @param kP The coefficient for the proportional term
 * @param kI The coefficient for the integral term
 * @param kD The coefficient for the derivative term
 * @param output The value to be set to the motors
 * @param pidMotors An array of up to 2 motors to apply the output value to
 */
typedef struct pidStruct {
	int* sensor;
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
	Ricemotor* pidMotors[2];
} Ricepid;

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
 * @param portTop (If not IME) The digital port on the Cortex which the top wire of the encoder is plugged into
 * @param portBot (If not IME) The digital port on the Cortex which the bottom wire of the encoder is plugged into
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
 * @param port The analog port on the Cortex which the potentiometer is plugged into
 * @param value The current value of the potentiometer
 * @param reversed Whether or not the potentiometer value should be inverted
 */
typedef struct RicepotStruct {
	unsigned char port;
	int value;
	int reversed;
} Ricepot;

/*
 * The Ricegyro is a wrapper for gyro use
 *
 * @param g The PROS Gyro base type
 * @param port The analog port on the Cortex which the potentiometer is plugged into
 * @param value The current value of the gyro
 * @param multiplier Sensitivity calibration for the gyro. Use 0 for the default value of 196
 */
typedef struct RicegyroStruct {
	Gyro g;
	unsigned char port;
	int value;
	unsigned short multiplier;
} Ricegyro;

/*
 * The Ricesolenoid is a wrapper for solenoid use
 *
 * @param port The digital port on the Cortex which the solenoid is plugged into
 * @param state The current state of the solenoid, either HIGH (extended) or LOW (retracted)
 * @param reversed Is the direction of the solenoid reversed? True or False
 */
typedef struct RicesolenoidStruct {
	unsigned char port;
	int state;
	int reversed;
} Ricesolenoid;

/*
 * The Motor Vector is a variable length array of Motor structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct motorVector {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricemotor* data[]; /* Pointer to actual array of data */
} ricemotorVector;

/*
 * The Pid Vector is a variable length array of Pid structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct ricepidVector {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricepid* data[]; /* Pointer to actual array of data */
} ricepidVector;

/*
 * The Ricencoder Vector is a variable length array of Ricencoder structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct ricencoderVector {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricencoder* data[]; /* Pointer to actual array of data */
} ricencoderVector;

/*
 * The Ricepot Vector is a variable length array of Ricepot structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct ricepotVector {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricepot* data[]; /* Pointer to actual array of data */
} ricepotVector;

/*
 * The Ricesolenoid Vector is a variable length array of Ricesolenoid structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct ricesolenoidVector {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricesolenoid* data[]; /* Pointer to actual array of data */
} ricesolenoidVector;

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
#define AUTOTURNTIME	7
#define AUTOCLAW		8
#define AUTOARMTIME		9

//Default Motor
Ricemotor* MOTDefault;

//Declaration of all possible Drivetrain motors
Ricemotor* MOTDTFront;
Ricemotor* MOTDTFrontRight;
Ricemotor* MOTDTFrontMidRight;
Ricemotor* MOTDTMidRight;
Ricemotor* MOTDTBackRight;
Ricemotor* MOTDTFrontLeft;
Ricemotor* MOTDTFrontMidLeft;
Ricemotor* MOTDTMidLeft;
Ricemotor* MOTDTBackLeft;
Ricemotor* MOTDTBack;

//Declaration of all possible Arm motors
//This is just to make the code more understandable when comparing an arm motor
//name to the physical motor on the robot
Ricemotor* MOTARMFront;
Ricemotor* MOTARMBack;
Ricemotor* MOTARMTop;
Ricemotor* MOTARMMiddle;
Ricemotor* MOTARMBottom;
Ricemotor* MOTARMLeft;
Ricemotor* MOTARMRight;
Ricemotor* MOTARMTopRight;
Ricemotor* MOTARMBottomRight;
Ricemotor* MOTARMTopLeft;
Ricemotor* MOTARMBottomLeft;
Ricemotor* MOTARMOuterLeft;
Ricemotor* MOTARMOuterRight;
Ricemotor* MOTARMInnerLeft;
Ricemotor* MOTARMInnerRight;

//Declaration of all possible Collector motors
Ricemotor* MOTCOL;
Ricemotor* MOTCOLLeft;
Ricemotor* MOTCOLRight;
Ricemotor* MOTCLAW;

ricemotorVector* MOTVector;
//motorVector* DTMotorVector;
//motorVector* ARMMotorVector;
//motorVector* COLMotorVector;

//Default Ricepid
Ricepid* PidDefault;

Ricepid* PidDTLeft;
Ricepid* PidDTRight;
Ricepid* PidARMLeft;
Ricepid* PidARMRight;
Ricepid* PidARMBottom;
Ricepid* PidARMTop;
Ricepid* PidARMFront;
Ricepid* PidARMTop;
Ricepid* PidARMBottom;

ricepidVector* PidVector;

//unsigned char IMEDTLEFT;
//unsigned char IMEDTRIGHT;
//unsigned char IMEARMLEFT;
//unsigned char IMEARMRIGHT;

//Default Ricencoder
Ricencoder* EncDefault;

Ricencoder* EncDTLeft;
Ricencoder* EncDTRight;
Ricencoder* EncARMLeft;
Ricencoder* EncARMRight;
Ricencoder* EncARMBottom;
Ricencoder* EncARMTop;

ricencoderVector* EncVector;

//Default Ricepot
Ricepot* PotDefault;

Ricepot* PotARMFront;
Ricepot* PotARMLeft;
Ricepot* PotARMRight;

ricepotVector* PotVector;

Ricegyro* gyro;

//Default Ricesolenoid
Ricesolenoid* SolDefault;

Ricesolenoid* SolClaw;

ricesolenoidVector* SolVector;

//Prototyping of RiceStruct initialization functions

Ricemotor* initRicemotor(unsigned char port, int reflected);

Ricepid* initRicepid(int* sensor, float kP, float kI, float kD, Ricemotor* motors[2]);

Ricencoder* initRicencoder(float ticksPerRev, int mult, int isIME, unsigned char imeAddress,
		unsigned char portTop, unsigned char portBot, bool reverse);

Ricencoder* initRicencoderIME(float ticksPerRev, int mult, unsigned char imeAddress,
		bool reverse);

Ricencoder* initRicencoderQUAD(float ticksPerRev, int mult, unsigned char portTop,
		unsigned char portBot, bool reverse);

Ricepot* initRicepot(unsigned char port, int reversed);

Ricegyro* initRicegyro(unsigned char port, unsigned short multiplier);

Ricesolenoid* initRicesolenoid(unsigned char port, int state, int reversed);

//Prototyping of all Vector-related functions

ricemotorVector* initRicemotorVector();

int ricemotorVectorAppend(ricemotorVector* vect, Ricemotor* element);

Ricemotor* ricemotorVectorGet(ricemotorVector* vect, int index);

ricepidVector* initRicepidVector();

int ricepidVectorAppend(ricepidVector* vect, Ricepid* element);

Ricepid* ricepidVectorGet(ricepidVector* vect, int index);

ricencoderVector* initRicencoderVector();

int ricencoderVectorAppend(ricencoderVector* vect, Ricencoder* element);

Ricencoder* ricencoderVectorGet(ricencoderVector* vect, int index);

ricepotVector* initRicepotVector();

int ricepotVectorAppend(ricepotVector* vect, Ricepot* element);

Ricepot* ricepotVectorGet(ricepotVector* vect, int index);

ricesolenoidVector* initRicesolenoidVector();

int ricesolenoidVectorAppend(ricesolenoidVector* vect, Ricesolenoid* element);

Ricesolenoid* ricesolenoidVectorGet(ricesolenoidVector* vect, int index);

//Prototyping of other functions

void riceBotInitializeIO();

void riceBotInitialize();

void getJoystickForDriveTrain();

void updatePid(Ricepid *pidLoop);

void updateRicencoder(Ricencoder *rc);

void updateRicepot(Ricepot *rp);

void updateRicegyro(Ricegyro *rg);

void autonomousTask(int instruction, int distance, int pow, long timeout);

int speedRegulator(int speed);

int normalize(int left, int right);

void DTStopMotors();

void IOTask(void *ignore);

void PidTask(void *ignore);

int max(int a, int b);

int min(int a, int b);

int max4(int a, int b, int c, int d);

#endif /* RICEBOT_H_ */
