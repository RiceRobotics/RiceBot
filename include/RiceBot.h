/*
 * Welcome to the RiceBot library!
 *
 * This library was written for use by the Rice University Vex U Robotics team.
 * All are welcome to use and modify the library, so long as due credit is given to the creator.
 *
 * If you have questions/comments/suggestions, email me at keikokaplan@gmail.com
 *
 * This library was written to be used with the Purdue Robotic Operating System.
 *
 * Style Notes:
 * 			All custom types are called Rice*, capitalized like objects in Java.
 * 			Rice* is considered one word, so Ricemotor, NOT RiceMotor.
 * 			Use camelCase for everything else.
 *
 * Created on: Aug 29, 2014
 * Author: 			Keiko Kaplan
 *
 * Contributors: Cannon Lewis
 * 						Lewis Lewis
 */

#ifndef RICEBOT_H_
#define RICEBOT_H_

#include <math.h>
#include <API.h>

/**********************************************************************************************
 * 														Definitions for Rice* types
 **********************************************************************************************/

/**
 * The Motor type serves as a wrapper to keep track of all data for each motor on the robot.
 *
 * @param port The motor port on the Cortex which the motor is plugged into
 * @param out The power output to the motor, between -127 and 127
 * @param reflected If the output to the motor should be reversed. -1 or 1
 */
typedef struct RicemotorStruct {
	unsigned char port;
	int out;
	int reflected;
} Ricemotor;

/**
 * The ricesensorDigital is a wrapper for generic cortex digital input
 *
 * @param port The digital port on the Cortex which the sensor is plugged into
 * @param state The current state of the sensor, either 1 or 0
 */
typedef struct ricesensorDigitalStruct {
	unsigned char port;
	int state;
} RicesensorDigital;

/**
 * The RicesensorAnalog is a wrapper for generic cortex analog input
 *
 * @param port The digital port on the Cortex which the sensor is plugged into
 * @param state The current state of the sensor, from 0 to 4095 to designate approximate input voltage
 * @param cal True if you want to calibrate the sensor
 */
typedef struct RicesensorAnalogStruct {
	unsigned char port;
	int state;
	bool cal;
} RicesensorAnalog;

/**
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

/**
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

/**
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

/**
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

/**
 * The Ricebutton is a wrapper for button and limit switch use
 *
 * @param port The digital port on the Cortex which the button is plugged into
 * @param state The current state of the button, either HIGH (Released) or LOW (Pressed)
 */
typedef struct RicebuttonStruct {
	unsigned char port;
	int state;
} Ricebutton;

/**
 * The Pid type contains all data for any individual pid loop we may wish to run.
 *
 * @param *sensor A pointer to the sensor's value field
 * @param running 1 if the Pid instance should be processed/set to motors, else 0
 * @param setPoint The target value for the loop
 * @param tolerance Acceptable variance from the setpoint
 * @param atSetpoint 1 if current is within tolerance of the setpoint
 * @param atSetpointTime How long we've been at setpoint
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
typedef struct RicepidStruct {
	int* sensor;
	int running;
	int setPoint;
	int tolerance;
	int atSetpoint;
	long atSetpointTime;
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

//TODO: Finalize type and add docstring
typedef struct RicelocationStruct {
	int xRaw;
	int yRaw;
	int x;
	int y;
	int angle;
} Ricelocation;

//TODO: Finalize type and add docstring
typedef struct RPSStruct {
	Ricelocation *currentLoc;
	int lastEncLeft;
	int lastEncRight;
} Ricerps;

//TODO: Finalize type and add docstring
typedef struct RiceautonTaskStruct {
	int index;
	int isRunning;
	long startTime;
	Ricemotor* motors[4];
} RiceautonTask;

/**********************************************************************************************
 * 													Global constants and variables
 **********************************************************************************************/

//Constants
#define ever ;;
#define MATH_PI 3.14159265358979323846

//Teams
#define RED -1
#define BLUE 1

//Control Style
#define CTTANKDRIVE  			0
#define CTARCADEDRIVE 		1
#define CTCHEEZYDRIVE  		2
#define CTMECANUMDRIVE  	3
#define CTHDRIVE  				4
#define CTHDRIVEARCADE 	5

//Defines how the analog joystick inputs translate to motor outputs (CT*)
int controlStyle;

//Drivetrain Styles
#define DTFOURWHEELS 	0
#define DTSIXWHEELS 	1
#define DTEIGHTWHEELS 	2
#define DTMECANUM 		3
#define DTHOLONOMIC 	4
#define DTSWERVE 		6
#define DTHDRIVE 7

//Defines the drivetrain installed on the robot (DT*)
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
Ricemotor* MOTDTHDrive;

RicesensorDigital* DigitalDefault;
RicesensorAnalog* AnalogDefault;

//Default Ricepid
Ricepid* PidDefault;

//Default Ricencoder
Ricencoder* EncDefault;
Ricencoder* EncDTLeft;
Ricencoder* EncDTRight;

//Default Ricepot
Ricepot* PotDefault;

Ricegyro* gyro;

//Default Ricesolenoid
Ricesolenoid* SolDefault;

//Default Ricebutton
Ricebutton* ButDefault;

//Robot Positioning System (RPS)
int rpsActive;
Ricelocation* startingLoc;
Ricelocation* currentLoc;
Ricelocation* targetLoc;
Ricerps* rps;

/**********************************************************************************************
 * 												Ricetype initialization function prototypes
 **********************************************************************************************/

/**
 * Initializes a Ricemotor type
 *
 * @param port The port on the Cortex which the motor is plugged into
 * @param reflected If the output to the motor should be reversed. -1 or 1
 *
 * @return The initialized and configured motor
 */
Ricemotor* initRicemotor(unsigned char port, int reflected);

/**
 * Initializes a generic digital sensor
 *
 * @param port The port on the Cortex which the sensor is plugged into
 * @return The initialized and configured digital sensor
 */
RicesensorDigital* initRicesensorDigital(unsigned char port);

/**
 * Initializes a generic analog sensor
 *
 * @param port The port on the Cortex which the sensor is plugged into
 * @param cal True if the sensor should be calibrated with its current value
 * @return The initialized and configured analog sensor
 */
RicesensorAnalog* initRicesensorAnalog(unsigned char port, bool cal);

/**
 * Initializes an encoder
 *
 * @param ticksPerRev The number of ticks per revolution of the encoder
 * 						627.2 for the 393 IME in high torque mode (factory default)
 * 						392 for the 393 IME in high speed mode
 * 						360 for the Quadrature Encoder
 * @param mult A multiplier to use as compensation for gear ratio
 * @param isIME 1 if IME, 0 if quad encoder
 * @param imeAddress The address of the IME, in order of the chain from the Cortex. Starts at 0
 * @param portTop (If not IME) The port on the Cortex which the top wire of the encoder is plugged into
 * @param portBot (If not IME) The port on the Cortex which the bottom wire of the encoder is plugged into
 * @param reverse Whether the Ricencoder should count in the reverse direction
 *
 * @return The initialized and configured Ricencoder
 */
Ricencoder* initRicencoder(float ticksPerRev, int mult, int isIME, unsigned char imeAddress,
		unsigned char portTop, unsigned char portBot, bool reverse);

/**
 * Initializes an IME encoder
 *
 * @param ticksPerRev The number of ticks per revolution of the encoder
 * 						627.2 for the 393 IME in high torque mode (factory default)
 * 						392 for the 393 IME in high speed mode
 * 						360 for the Quadrature Encoder
 * @param mult A multiplier to use as compensation for gear ratio
 * @param imeAddress The address of the IME, in order of the chain from the Cortex. Starts at 0
 * @param reverse Whether the Ricencoder should count in the reverse direction
 *
 * @return The initialized and configured Ricencoder
 */
Ricencoder* initRicencoderIME(float ticksPerRev, int mult, unsigned char imeAddress,
		bool reverse);

/**
 * Initializes a quadrature encoder
 *
 * @param ticksPerRev The number of ticks per revolution of the encoder
 * 						627.2 for the 393 IME in high torque mode (factory default)
 * 						392 for the 393 IME in high speed mode
 * 						360 for the Quadrature Encoder
 * @param mult A multiplier to use as compensation for gear ratio
 * @param portTop (If not IME) The port on the Cortex which the top wire of the encoder is plugged into
 * @param portBot (If not IME) The port on the Cortex which the bottom wire of the encoder is plugged into
 * @param reverse Whether the Ricencoder should count in the reverse direction
 *
 * @return The initialized and configured Ricencoder
 */
Ricencoder* initRicencoderQUAD(float ticksPerRev, int mult, unsigned char portTop,
		unsigned char portBot, bool reverse);

/**
 * Initializes a Ricepot
 *
 * @param port The port on the Cortex which the potentiometer is plugged into
 * @param reversed 1 if normal, -1 if inverted
 *
 * @return The initialized and configured Ricepot
 */
Ricepot* initRicepot(unsigned char port, int reversed);

/**
 * Initializes a Ricegyro
 *
 * @param port The port on the Cortex which the gyro is plugged into
 * @param multiplier Sensitivity calibration for the gyro. Use 0 for the default value of 196
 */
Ricegyro* initRicegyro(unsigned char port, unsigned short multiplier);

/**
 * Initializes a Ricesolenoid
 *
 * @param port The digital port on the Cortex which the solenoid is plugged into
 * @param state The initial state of the piston (HIGH for extended, LOW for retracted)
 * @param reversed True if the behavior of the solenoid is reversed, else False
 */
Ricesolenoid* initRicesolenoid(unsigned char port, int state, int reversed);

/**
 * Initializes a Ricebutton
 *
 * @param port The port on the Cortex which the button or limit switch is plugged into
 *
 * @return The initialized and configured Ricebutton
 */
Ricebutton* initRicebutton(unsigned char port);

/**
 * Initializes a Pid type
 *
 * @param *sensor A pointer to the sensor's value field
 * @param tolerance Acceptable variance from the setpoint
 * @param kP The coefficient for the proportional term
 * @param kI The coefficient for the integral term
 * @param kD The coefficient for the derivative term
 * @param motors[2] An array of up to 2 motors to use as output
 *
 * @return The initialized and configured Pid
 */
Ricepid* initRicepid(int* sensor, int tolerance, float kP, float kI, float kD, Ricemotor* motors[2]);

Ricelocation* initRicelocation(int x, int y, int angle);

Ricerps* initRPS(Ricelocation* loc);

RiceautonTask* initRiceautonTask(int index, Ricemotor* motors[4]);

/**********************************************************************************************
 * 												Protyping of other functions
 **********************************************************************************************/

void riceBotInitializeIO();

/**
 * Call this from the default Initialize function.
 * After, be sure to reinitialize each Ricetype you will be using on your robot.
 */
void riceBotInitialize();

/**
 * Checks joystick input and sets all Motor structs to appropriate output
 */
void getJoystickForDriveTrain();

/**
 * Updates a PID loop based on the value of a sensor
 *
 * @param *pidLoop A pointer to the pid struct to update
 * @param current The current value of the relevant sensor to use with the pid loop
 *
 */
void updatePid(Ricepid *pidLoop);

/**
 * Updates the value of any Ricencoder based on a pointer to the struct
 *
 * @param *rc A pointer to the Ricencoder
 */
void updateRicencoder(Ricencoder *rc);

/**
 * Updates the value of any Ricepot based on a pointer to the struct
 *
 * @param *rg A pointer to the Ricepot
 */
void updateRicepot(Ricepot *rp);

/**
 * Updates the value of any Ricegyro based on a pointer to the struct
 *
 * @param *rg A pointer to the Ricegyro
 */
void updateRicegyro(Ricegyro *rg);

/**
 * Updates the value of any Ricesolenoid based on a pointer to the struct
 *
 * @param rs A pointer to the Ricesolenoid
 */
void updateRicesolenoid(Ricesolenoid *rs);

/**
 * Updates the value of any Ricebutton based on a pointer to the struct
 *
 * @param rb A pointer to the Ricebutton
 */
void updateRicebutton(Ricebutton *rb);

//TODO
void updateRPS(Ricerps *rps, int encLeft, int encRight);

void updateRicesensorDigital(RicesensorDigital *blah);

void updateRicesensorAnalog(RicesensorAnalog *blahh);

void resetRicencoder();

/**
 * Runs an instruction, to be used during autonomous mode
 *
 * @param instruction The autonomous instruction to execute. Options are:
 * 						AUTODRIVEBASIC
 *						AUTODRIVEGYRO
 *						AUTOTURNBASIC
 *						AUTOTURNPID
 *						AUTOARMPID
 *						AUTOCOLLECTORS
 *						AUTODRIVETIME
 *						AUTOTURNTIME
 *						AUTOCLAW
 *						AUTOARMTIME
 * @param distance The distance, in inches or degrees, for the instruction. NULL for time commands.
 * @param pow The starting value for output power. NULL for max power
 * @param timeout Instruction will end regardless after this time. Duration for time commands
 */
void autonomousTask(int instruction, int distance, int pow, long timeout);

/**
 * Ensures a value is inside the acceptable bounds for a motor output
 *
 * @param speed The desired speed to set to a motor
 *
 * @return An integer adjusted to -127 <= value <= 127
 */
int speedRegulator(int speed);

/**
 * Normalize the left and right sides of the drivetrain
 *
 * @param left The value for the left side of the drivetrain
 * @param right The value for the right side of the drivetrain
 *
 * @return A multiplier for the motor output
 */
float normalize(int left, int right);

/**
 * Shuts down all the motors
 */
void DTStopMotors();

/**
 * A thread to handle all input and output of sensors and motors
 */
void IOTask(void *ignore);

/**
 * A thread to update any PID control that may be in use
 */
void PidTask(void *ignore);

/**
 * A thread for anything else we might need
 */
void miscTask(void *ignore);

/**
 * Determines which of two numbers is larger
 *
 * @param a The first number
 * @param b The second number
 *
 * @return Whichever number is larger
 */
int max(int a, int b);

/**
 * Determines which of two numbers is smaller
 *
 * @param a The first number
 * @param b The second number
 *
 * @return Whichever number is smaller
 */
int min(int a, int b);

/**
 * Determines which of 4 numbers is the largest
 *
 * @param a The first number
 * @param b The second number
 * @param c The third number
 * @param d The fourth number
 *
 * @return The largest number
 */
int max4(int a, int b, int c, int d);

/**
 * The Motor Vector is a variable length array of Motor structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct RicemotorVectorStruct {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricemotor* data[]; /* Pointer to actual array of data */
} RicemotorVector;

/**
 * The Pid Vector is a variable length array of Pid structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct RicepidVectorStruct {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricepid* data[]; /* Pointer to actual array of data */
} RicepidVector;

/**
 * The Ricencoder Vector is a variable length array of Ricencoder structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct RicencoderVectorStruct {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricencoder* data[]; /* Pointer to actual array of data */
} RicencoderVector;

/**
 * The Ricepot Vector is a variable length array of Ricepot structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct RicepotVectorStruct {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricepot* data[]; /* Pointer to actual array of data */
} RicepotVector;

/**
 * The Ricesolenoid Vector is a variable length array of Ricesolenoid structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct RicesolenoidVectorStruct {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricesolenoid* data[]; /* Pointer to actual array of data */
} RicesolenoidVector;

/**
 * The Ricebutton Vector is a variable length array of Ricebutton structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct RicebuttonVectorStruct {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Ricebutton* data[]; /* Pointer to actual array of data */
} RicebuttonVector;

/**
 * The ricesensorDigital Vector is a variable length array of ricesensorDigital structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct RiceseonsorDigitalVectorStruct {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	RicesensorDigital* data[]; /* Pointer to actual array of data */
} RicesensorDigitalVector;

/**
 * The RicesensorAnalog Vector is a variable length array of RicesensorAnalog structs
 *
 * @param elem_total  Number of elements currently allocated for, initialized to 10
 * @param elem_current Current number of elements stored
 * @param data Pointer to actual array of data
 */
typedef struct RicesensorAnalogVectorStruct {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	RicesensorAnalog* data[]; /* Pointer to actual array of data */
} RicesensorAnalogVector;

RicemotorVector* MOTVector;

RicepidVector* PidVector;
RicencoderVector* EncVector;
RicepotVector* PotVector;
RicesolenoidVector* SolVector;
RicebuttonVector* ButVector;
RicesensorDigitalVector* DigitalVector;
RicesensorAnalogVector* AnalogVector;

//Prototyping of all Vector-related functions

RicemotorVector* initRicemotorVector();

int ricemotorVectorAppend(RicemotorVector* vect, Ricemotor* element);

Ricemotor* ricemotorVectorGet(RicemotorVector* vect, int index);

RicepidVector* initRicepidVector();

int ricepidVectorAppend(RicepidVector* vect, Ricepid* element);

Ricepid* ricepidVectorGet(RicepidVector* vect, int index);

RicencoderVector* initRicencoderVector();

int ricencoderVectorAppend(RicencoderVector* vect, Ricencoder* element);

Ricencoder* ricencoderVectorGet(RicencoderVector* vect, int index);

RicepotVector* initRicepotVector();

int ricepotVectorAppend(RicepotVector* vect, Ricepot* element);

Ricepot* ricepotVectorGet(RicepotVector* vect, int index);

RicesolenoidVector* initRicesolenoidVector();

int ricesolenoidVectorAppend(RicesolenoidVector* vect, Ricesolenoid* element);

Ricesolenoid* ricesolenoidVectorGet(RicesolenoidVector* vect, int index);

RicebuttonVector* initRicebuttonVector();

int ricebuttonVectorAppend(RicebuttonVector* vect, Ricebutton* element);

Ricebutton* ricebuttonVectorGet(RicebuttonVector* vect, int index);

RicesensorAnalogVector* initRicesensorAnalogVector();

int ricesensorAnalogVectorAppend(RicesensorAnalogVector* vect, RicesensorAnalog* element);

RicesensorAnalog* ricesensorAnalogVectorGet(RicesensorAnalogVector* vect, int index);

RicesensorDigitalVector* initRicesensorDigitalVector();

int ricesensorDigitalVectorAppend(RicesensorDigitalVector* vect, RicesensorDigital* element);

RicesensorDigital* ricesensorDigitalVectorGet(RicesensorDigitalVector* vect, int index);

#endif /* RICEBOT_H_ */
