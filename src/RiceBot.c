/*
 * Welcome to the RiceBot library!
 *
 * This library was written for use by the Rice University Vex U Robotics team.
 * All are welcome to use and modify the library, so long as due credit is given to the creator.
 * If you have questions/comments/suggestions, email me at Keiko.F.Kaplan@rice.edu
 *
 * This library was written to be used with the Purdue Robotic Operating System.
 *
 * Author: Keiko Kaplan
 */

#include "RiceBot.h"
#include <API.h>

void riceBotInitializeIO() {
//  SolVector = initRicesolenoidVector();
//  SolDefault = initRicesolenoid(0, LOW, false);
//  SolClaw = SolDefault;
}

/**
 * Call this from the default Initialize function.
 * After, be sure to reinitialize each motor you will be using on your robot.
 */
void riceBotInitialize() {

	printf("riceBotInitialize\n\r");

	MOTVector = initRicemotorVector();
	PidVector = initRicepidVector();
	EncVector = initRicencoderVector();
	PotVector = initRicepotVector();
	ButVector = initRicebuttonVector();

	MOTDefault = initRicemotor(0, 1);
	MOTDTFront = MOTDefault;
	MOTDTFrontRight = MOTDefault;
	MOTDTFrontMidRight = MOTDefault;
	MOTDTMidRight = MOTDefault;
	MOTDTBackRight = MOTDefault;
	MOTDTFrontLeft = MOTDefault;
	MOTDTFrontMidLeft = MOTDefault;
	MOTDTMidLeft = MOTDefault;
	MOTDTBackLeft = MOTDefault;
	MOTDTBack = MOTDefault;
	MOTDTHDrive = MOTDefault;

//	MOTARMFront = MOTDefault;
//	MOTARMBack = MOTDefault;
//	MOTARMTop = MOTDefault;
//	MOTARMMiddle = MOTDefault;
//	MOTARMBottom = MOTDefault;
//	MOTARMLeft = MOTDefault;
//	MOTARMRight = MOTDefault;
//	MOTARMTopRight = MOTDefault;
//	MOTARMBottomRight = MOTDefault;
//	MOTARMTopLeft = MOTDefault;
//	MOTARMBottomLeft = MOTDefault;
//	MOTARMOuterLeft = MOTDefault;
//	MOTARMOuterRight = MOTDefault;
//	MOTARMInnerLeft = MOTDefault;
//	MOTARMInnerRight = MOTDefault;

//	MOTCOL = MOTDefault;
//	MOTCOLLeft = MOTDefault;
//	MOTCOLRight = MOTDefault;
//	MOTCLAW = MOTDefault;
//	MOTConveyor = MOTDefault;

	Ricemotor* array[2] = {MOTDefault, MOTDefault};
	PidDefault = initRicepid(0, 0, 0, 0, array);
//	PidDTLeft = PidDefault;
//	PidDTRight = PidDefault;
//	PidARMLeft = PidDefault;
//	PidARMRight = PidDefault;
//	PidARMBottom = PidDefault;
//	PidARMTop = PidDefault;
//	PidARMFront = PidDefault;
//	PidARMTop = PidDefault;
//	PidARMBottom = PidDefault;

	EncDefault = initRicencoder(0, 0, 0, 0, 0, 0, false);
//	EncDTLeft = EncDefault;
//	EncDTRight = EncDefault;
//	EncARMLeft = EncDefault;
//	EncARMRight = EncDefault;
//	EncARMBottom = EncDefault;
//	EncARMTop = EncDefault;
//	EncARMFront = EncDefault;

	PotDefault = initRicepot(0, 0);
//	PotARMFront = PotDefault;
//	PotARMLeft = PotDefault;
//	PotARMRight = PotDefault;

	ButDefault = initRicebutton(0);
//	ButConLeft = ButDefault;
//	ButConRight = ButDefault;
//	ButARMBase = ButDefault;
//	ButARMFrontLeft = ButDefault;
//	ButARMFrontRight = ButDefault;
	
//	imeInitializeAll();
	printf("Initialization complete\n\r");
}

/**
 * Initializes a Motor type
 *
 * @param port The port on the Cortex which the motor is plugged into
 * @param reflected If the output to the motor should be reversed. -1 or 1
 *
 * @return The initialized and configured motor
 */
Ricemotor* initRicemotor(unsigned char port, int reflected) {
	Ricemotor *m = malloc(sizeof(Ricemotor));
	m->port = port;
	m->out = 0;
	m->reflected = reflected;

	ricemotorVectorAppend(MOTVector, m);
	return m;
}

/**
 * Initializes a Pid type
 *
 * @param *sensor A pointer to the sensor's value field
 * @param kP The coefficient for the proportional term
 * @param kI The coefficient for the integral term
 * @param kD The coefficient for the derivative term
 * @param motors[2] An array of up to 2 motors to use as output
 *
 * @return The initialized and configured Pid
 */
Ricepid* initRicepid(int* sensor, float kP, float kI, float kD, Ricemotor* motors[2]) {
	Ricepid *p = malloc(sizeof(Ricepid));
	p->sensor = sensor;
	p->running = 0;
	p->setPoint = 0;
	p->current = 0;
	p->error = 0;
	p->lastError = 0;
	p->integral = 0;
	p->derivative = 0;
	p->kP = kP;
	p->kI = kI;
	p->kD = kD;
	p->output = 0;
	for(int i = 0; i < 2; i++) {
		p->pidMotors[i] = motors[i];
	}
	ricepidVectorAppend(PidVector, p);
	return p;
}

/**
 * The Ricencoder contains data for either an IME or a quadrature encoder
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
		unsigned char portTop, unsigned char portBot, bool reverse) {
	Ricencoder *r = malloc(sizeof(Ricencoder));
	r->rawValue = 0;
	r->ticksPerRev = ticksPerRev;
	r->mult = mult;
	r->adjustedValue = 0;
	r->isIME = isIME;
	r->imeAddress = imeAddress;
	r->portTop = portTop;
	r->portBot = portBot;
	r->reverse = reverse;
	if(!isIME) {
		r->enc = encoderInit(portTop, portBot, reverse);
	}
	ricencoderVectorAppend(EncVector, r);
	return r;
}

Ricencoder* initRicencoderIME(float ticksPerRev, int mult, unsigned char imeAddress,
		bool reverse) {
	Ricencoder *r = malloc(sizeof(Ricencoder));
	r->rawValue = 0;
	r->ticksPerRev = ticksPerRev;
	r->mult = mult;
	r->adjustedValue = 0;
	r->isIME = 1;
	r->imeAddress = imeAddress;
	r->portTop = 0;
	r->portBot = 0;
	r->reverse = reverse;
	ricencoderVectorAppend(EncVector, r);
	return r;
}

Ricencoder* initRicencoderQUAD(float ticksPerRev, int mult, unsigned char portTop,
		unsigned char portBot, bool reverse) {
	Ricencoder *r = malloc(sizeof(Ricencoder));
	r->rawValue = 0;
	r->ticksPerRev = ticksPerRev;
	r->mult = mult;
	r->adjustedValue = 0;
	r->isIME = 0;
	r->imeAddress = 0;
	r->portTop = portTop;
	r->portBot = portBot;
	r->reverse = reverse;
	r->enc = encoderInit(portTop, portBot, reverse);
	ricencoderVectorAppend(EncVector, r);
	return r;
}

/**
 * Initializes a Ricepot
 *
 * @param port The port on the Cortex which the potentiometer is plugged into
 * @param reversed 1 if normal, -1 if inverted
 *
 * @return The initialized and configured Ricepot
 */
Ricepot* initRicepot(unsigned char port, int reversed) {
	analogCalibrate(port);
	Ricepot *r = malloc(sizeof(Ricepot));
	r->port = port;
	r->value = 0;
	r->reversed = reversed;
	ricepotVectorAppend(PotVector, r);
	return r;
}

/**
 * Initializes a Ricegyro
 *
 * @param port The port on the Cortex which the gyro is plugged into
 * @param multiplier Sensitivity calibration for the gyro. Use 0 for the default value of 196
 */
Ricegyro* initRicegyro(unsigned char port, unsigned short multiplier) {
	Ricegyro *gyro = malloc(sizeof(Ricegyro));
	gyro->port = port;
	gyro->multiplier = multiplier;
	printf("Ricegyro Initialization (no touchie the robutt)\n\r");
	gyro->g = gyroInit(gyro->port, gyro->multiplier);
	delay(1000);
	printf("Ricegyro Initialized with minimal casualties\n\r");
	gyroReset(gyro->g);
	gyro->value = 0;
	return gyro;
}

/**
 * Initializes a Ricesolenoid
 *
 * @param port The digital port on the Cortex which the solenoid is plugged into
 * @param state The initial state of the piston (HIGH for extended, LOW for retracted)
 * @param reversed True if the behavior of the solenoid is reversed, else False
 */
Ricesolenoid* initRicesolenoid(unsigned char port, int state, int reversed) {
	Ricesolenoid *sol = malloc(sizeof(Ricesolenoid));
	sol->port = port;
//	sol->state = reversed ? state;
	sol->reversed = reversed;
	ricesolenoidVectorAppend(SolVector, sol);
	pinMode(sol->port, OUTPUT);

	digitalWrite(sol->port, sol->state);
	return sol;
}

/**
 * Initializes a Ricebutton
 *
 * @param port The port on the Cortex which the button or limit switch is plugged into
 *
 * @return The initialized and configured Ricebutton
 */
Ricebutton* initRicebutton(unsigned char port) {
	Ricebutton *r = malloc(sizeof(Ricebutton));
	r->port = port;
	pinMode(port, INPUT);
	r->state = digitalRead(port);
	ricebuttonVectorAppend(ButVector, r);
	return r;
}

/**
 * Checks joystick input and sets all Motor structs to appropriate output
 */
void getJoystickForDriveTrain() {
	//	printf("Get Joystick\n\r");
	int x1 = joystickGetAnalog(1, 4);
	int y1 = joystickGetAnalog(1, 3);
	int x2 = joystickGetAnalog(1, 1);
	int y2 = joystickGetAnalog(1, 2);
	int left, right;
	float norm;

	//	printf("Joysticks Gotten\n\r");

	switch(controlStyle) {
	case CTTANKDRIVE:
		MOTDTFrontLeft->out = y1;
		MOTDTFrontMidLeft->out = y1;
		MOTDTMidLeft->out = y1;
		MOTDTBackLeft->out = y1;

		MOTDTFrontRight->out = y2;
		MOTDTFrontMidRight->out = y2;
		MOTDTMidRight->out = y2;
		MOTDTBackRight->out = y2;
		break;
	case CTARCADEDRIVE:
		left = y1 + x1;
		right = y1 - x1;
		norm = normalize(left, right);

		MOTDTFrontLeft->out = left * norm;
		MOTDTFrontMidLeft->out = left * norm;
		MOTDTMidLeft->out = left * norm;
		MOTDTBackLeft->out = left * norm;

		MOTDTFrontRight->out = right * norm;
		MOTDTFrontMidRight->out = right * norm;
		MOTDTMidRight->out = right * norm;
		MOTDTBackRight->out = (right * norm);
		break;
	case CTCHEEZYDRIVE:
		left = y1 + x2;
		right = y1 - x2;
		norm = normalize(left, right);

		//		motorVectorGet(DTMotors, 0)->out = left * norm;
		//		MOTDTFrontLeft->out = left * norm;
		MOTDTFrontLeft->out = left * norm;
		//		printf("1");
		//		MOTDTFrontMidLeft->out = left * norm;
		//		MOTDTMidLeft->out = left * norm;
		MOTDTBackLeft->out = left * norm;
		//		printf("2");

		MOTDTFrontRight->out = right * norm;
		//		printf("3");
		//		MOTDTFrontMidRight->out = right * norm;
		//		MOTDTMidRight->out = right * norm;
		MOTDTBackRight->out = (right * norm);
		//		printf("4");
		//		printf("%d, %d\n\r", MOTDTFrontLeft->out, (motorVectorGet(DTMotors, 0))->out);
		break;

	case CTMECANUMDRIVE:
		MOTDTFrontLeft->out = y1 + x2 + x1;
		MOTDTBackLeft->out = y1 + x2 - x1;

		MOTDTFrontRight->out = y1 - x2 - x1;
		MOTDTBackRight->out = y1 - x2 + x1;
		break;
	case CTHDRIVE:
	default:
		break;
	}
}

/**
 * Updates a PID loop based on the value of a sensor
 *
 * @param *pidLoop A pointer to the pid struct to update
 * @param current The current value of the relevant sensor to use with the pid loop
 *
 */
void updatePid(Ricepid *pidLoop) {
	if (pidLoop->running) {
		pidLoop->current = *(pidLoop->sensor);
		pidLoop->error = pidLoop->setPoint - pidLoop->current;
		pidLoop->integral += pidLoop->error;
		pidLoop->derivative = pidLoop->lastError - pidLoop->error;

		pidLoop->output = speedRegulator((pidLoop->error * pidLoop->kP) +
				(pidLoop->integral * pidLoop->kI) + (pidLoop->derivative * pidLoop->kD));

		pidLoop->lastError = pidLoop->error;
		for(int i = 0; i < 2; i++) {
			pidLoop->pidMotors[i]->out = pidLoop->output;
		}
	}
}

/**
 * Updates the value of any Ricencoder based on a pointer to the struct
 *
 * @param *rc A pointer to the Ricencoder struct
 */
void updateRicencoder(Ricencoder *rc) {
	if(rc->isIME) {
		imeGet(rc->imeAddress, &rc->rawValue);
	}
	else {
		rc->rawValue = encoderGet(rc->enc);
	}
	rc->adjustedValue = rc->rawValue * rc->mult * (rc->reverse ? -1 : 1);
}

/**
 * Updates the value of any Ricepot based on a pointer to the struct
 *
 * @param *rg A pointer to the Ricepot struct
 */
void updateRicepot(Ricepot *rp) {
	rp->value = analogReadCalibrated(rp->port) * rp->reversed;
}

/**
 * Updates the value of any Ricegyro based on a pointer to the struct
 *
 * @param *rg A pointer to the Ricegyro struct
 */
void updateRicegyro(Ricegyro *rg) {
	if(rg != NULL) {
		rg->value = gyroGet(rg->g);
	}
}

void updateRicebutton(Ricebutton *rb) {
	if(rb != NULL) {
		rb->state = digitalRead(rb->port);
	}
}

void resetRicencoder() {
	for(int i = 1; i < EncVector->elem_current; i++) {
		imeReset(ricencoderVectorGet(EncVector, i)->imeAddress);
		updateRicencoder(ricencoderVectorGet(EncVector, i));
	}
}
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
void autonomousTask(int instruction, int distance, int pow, long timeout) {
	int target;
	long startTime = millis();

	int power[2];
	power[1] = (pow == NULL) ? 127 : pow;
	power[0] = power[1];

//	int currentEnc[2] = {EncDTLeft->adjustedValue, EncDTRight->adjustedValue};

	switch(instruction) {
	case AUTODRIVETIME:
		while(millis() < startTime + timeout) {
			MOTDTFrontRight->out = power[1];
			MOTDTFrontMidRight->out = power[1];
			MOTDTMidRight->out = power[1];
			MOTDTBackRight->out = power[1];
			MOTDTFrontLeft->out = power[0];
			MOTDTFrontMidLeft->out = power[0];
			MOTDTMidLeft->out = power[0];
			MOTDTBackLeft->out = power[0];
		}
		break;
	case AUTODRIVEBASIC:
//		target = EncDTLeft->ticksPerRev / (4 * MATH_PI) * distance;
//
//		while(currentEnc[1] < target && millis() < startTime + timeout) {
//			if(abs(currentEnc[1] - currentEnc[0]) > 50) {
//				if(currentEnc[0] > currentEnc[1]) {
//					power[0] = speedRegulator(power[0] - 2);
//				} else if(currentEnc[0] < currentEnc[1]) {
//					power[0] = speedRegulator(power[0] + 2);
//				}
//			}

			MOTDTFrontRight->out = power[1];
			MOTDTFrontMidRight->out = power[1];
			MOTDTMidRight->out = power[1];
			MOTDTBackRight->out = power[1];
			MOTDTFrontLeft->out = power[0];
			MOTDTFrontMidLeft->out = power[0];
			MOTDTMidLeft->out = power[0];
			MOTDTBackLeft->out = power[0];

//			delay(20);
//			currentEnc[0] = EncDTLeft->adjustedValue;
//			currentEnc[1] = EncDTRight->adjustedValue;
//		}
		break;
	case AUTOTURNBASIC:
		target = gyro->value + distance;
		if(gyro->value < target) {		//Left Turn
			while(gyro->value < target && millis() < startTime + timeout) {
				MOTDTFrontRight->out = pow;
				MOTDTFrontMidRight->out = pow;
				MOTDTMidRight->out = pow;
				MOTDTBackRight->out = pow;
				MOTDTFrontLeft->out = -pow;
				MOTDTFrontMidLeft->out = -pow;
				MOTDTMidLeft->out = -pow;
				MOTDTBackLeft->out = -pow;
			}
		}
		else if(gyro->value > target) {	//Right Turn
			while(gyro->value > target && millis() < startTime + timeout) {
				MOTDTFrontRight->out = -pow;
				MOTDTFrontMidRight->out = -pow;
				MOTDTMidRight->out = -pow;
				MOTDTBackRight->out = -pow;
				MOTDTFrontLeft->out = pow;
				MOTDTFrontMidLeft->out = pow;
				MOTDTMidLeft->out = pow;
				MOTDTBackLeft->out = pow;
			}
		}
		break;
	case AUTODRIVEGYRO:
//		target = EncDTLeft->ticksPerRev / (4 * MATH_PI) * distance;
//		int targetGyro = gyro->value;

//		while(currentEnc[0] < target && millis() < startTime + timeout) {
//			if(abs(gyro->value - targetGyro) > 10) {				//If gyro is outside of tolerance from start orientation
//				if(gyro->value > targetGyro) {					//Too far CCW
//					power[0] = speedRegulator(power[0] + 2);
//					power[1] = speedRegulator(power[1] - 2);
//				} else if(gyro->value < targetGyro) {			//Too far CW
//					power[0] = speedRegulator(power[0] - 2);
//					power[1] = speedRegulator(power[1] + 2);
//				}
//			}
//
//			MOTDTFrontRight->out = power[1];
//			MOTDTFrontMidRight->out = power[1];
//			MOTDTMidRight->out = power[1];
//			MOTDTBackRight->out = power[1];
//			MOTDTFrontLeft->out = power[0];
//			MOTDTFrontMidLeft->out = power[0];
//			MOTDTMidLeft->out = power[0];
//			MOTDTBackLeft->out = power[0];
//
//			delay(20);
////			currentEnc[0] = EncDTLeft->adjustedValue;
////			currentEnc[1] = EncDTRight->adjustedValue;
//		}
		break;
	case AUTOCOLLECTORS:
		if(timeout == NULL) {
//			MOTCOL->out = pow;
//			MOTCOLLeft->out = pow;
//			MOTCOLRight->out = pow;
		}
		else {
			while (millis() < startTime + timeout) {
//				MOTCOL->out = pow;
//				MOTCOLLeft->out = pow;
//				MOTCOLRight->out = pow;
			}
//			MOTCOL->out = 0;
//			MOTCOLLeft->out = 0;
//			MOTCOLRight->out = 0;
		}
		break;
	case AUTOARMTIME:
//		PidARMLeft->running = 0;
//		PidARMRight->running = 0;
//		if(timeout == NULL) {
//			MOTARMBottomLeft->out = pow;
//			MOTARMBottomRight->out = pow;
//		}
//		else {
//			while (millis() < startTime + timeout) {
//				MOTARMBottomLeft->out = pow;
//				MOTARMBottomRight->out = pow;
//			}
//			PidARMLeft->running = 1;
//			PidARMRight->running = 1;
//			PidARMLeft->setPoint = EncARMLeft->adjustedValue + 60;
//			PidARMRight->setPoint = EncARMRight->adjustedValue + 60;
//		}
		break;
	case AUTOCLAW:
//		if(timeout == NULL) {
//			MOTCLAW->out = pow;
//		}
//		else {
//			while (millis() < startTime + timeout) {
//				MOTCLAW->out = pow;
//			}
//			MOTCLAW->out = 0;
//		}
		break;
	case AUTOTURNTIME:
		target = distance;
		if(target > 0) {		//Left Turn
			while(millis() < startTime + timeout) {
				MOTDTFrontRight->out = pow;
				MOTDTFrontMidRight->out = pow;
				MOTDTMidRight->out = pow;
				MOTDTBackRight->out = pow;
				MOTDTFrontLeft->out = -pow;
				MOTDTFrontMidLeft->out = -pow;
				MOTDTMidLeft->out = -pow;
				MOTDTBackLeft->out = -pow;
			}
		}
		else if(target < 0) {	//Right Turn
			while(millis() < startTime + timeout) {
				MOTDTFrontRight->out = -pow;
				MOTDTFrontMidRight->out = -pow;
				MOTDTMidRight->out = -pow;
				MOTDTBackRight->out = -pow;
				MOTDTFrontLeft->out = pow;
				MOTDTFrontMidLeft->out = pow;
				MOTDTMidLeft->out = pow;
				MOTDTBackLeft->out = pow;
			}
		}
		break;
	default:
		break;
	}
	DTStopMotors();
}

/**
 * Ensures a value is inside the acceptable bounds for a motor output
 *
 * @param speed The desired speed to set to a motor
 *
 * @return An integer adjusted to -127 <= value <= 127
 */
int speedRegulator(int speed) {
	if(speed > 127) {
		return 127;
	} else if(speed < -127) {
		return -127;
	} else {
		return speed;
	}
}

float normalize(int left, int right) {
	float norm = 1;
	if(max(left, right) > 127) {
		norm = 127.0 / max(left, right);
	}
	return norm;
}

void DTStopMotors() {
	MOTDTFrontRight->out = 0;
	MOTDTFrontMidRight->out = 0;
	MOTDTMidRight->out = 0;
	MOTDTBackRight->out = 0;
	MOTDTFrontLeft->out = 0;
	MOTDTFrontMidLeft->out = 0;
	MOTDTMidLeft->out = 0;
	MOTDTBackLeft->out = 0;
	MOTDTHDrive->out = 0;
}

/**
 * Determines which of two numbers is larger
 *
 * @param a The first number
 * @param b The second number
 *
 * @return Whichever number is larger
 */
int max(int a, int b) {
	if(a < b) {
		return b;
	}
	return a;
}

/**
 * Determines which of two numbers is smaller
 *
 * @param a The first number
 * @param b The second number
 *
 * @return Whichever number is smaller
 */
int min(int a, int b) {
	if(a > b) {
		return b;
	}
	return a;
}

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
int max4(int a, int b, int c, int d) {
	return max(max(max(a, b), c), d);
}

void IOTask(void *ignore) {
	while(1) {
		//Update DT Motors
		for(int i = 1; i < MOTVector->elem_current; i++) {
			motorSet(ricemotorVectorGet(MOTVector, i)->port, ricemotorVectorGet(MOTVector, i)->out * ricemotorVectorGet(MOTVector, i)->reflected);
		}
		for(int i = 1; i < EncVector->elem_current; i++) {
			updateRicencoder(ricencoderVectorGet(EncVector, i));
		}
		for(int i = 1; i < PotVector->elem_current; i++) {
			updateRicepot(ricepotVectorGet(PotVector, i));
		}
		for(int i = 1; i < ButVector->elem_current; i++) {
			updateRicebutton(ricebuttonVectorGet(ButVector, i));
		}
		
		updateRicegyro(gyro);
		delay(10);
	}
}

void PidTask(void *ignore) {
	while(1) {
		for(int i = 1; i < PidVector->elem_current; i++) {
			updatePid(ricepidVectorGet(PidVector, i));
		}
		delay(20);
	}
}

/**
 * Initializes a vector
 */
ricemotorVector* initRicemotorVector() {
	ricemotorVector* vect = malloc(sizeof(vect->elem_current) + sizeof(vect->elem_total) + 10*(sizeof(Ricemotor*)));
	vect->elem_total = 14;
	vect->elem_current = 0;

	return vect;
}

/**
 * Adds an element to the vector
 *
 * @param vect A pointer to the destination vector
 * @param element A pointer to the new element
 *
 * @return 1 if successful and 0 otherwise
 */
int ricemotorVectorAppend(ricemotorVector* vect, Ricemotor* element) {
	vect->data[vect->elem_current] = element;
	vect->elem_current++;
	if(vect->elem_current >= vect->elem_total) {
		delay(500);
		printf("Motor Realloc!\n\r");
		delay(1000);
		Ricemotor* new_data = realloc(vect->data, sizeof(vect->elem_current) + sizeof(vect->elem_total) +
				(vect->elem_total*2*sizeof(Ricemotor*)));
		if(new_data) {
			*(vect->data) = new_data;
			vect->elem_total *= 2;
		} else {
			printf("Error allocating memory");
			free(vect->data);
			return 0;
		}
	}
	return 1;
}

/**
 * Returns the element at a given index
 *
 * @param vect A pointer to the destination vector
 * @param index The index of the element to retrieve
 *
 * @return -1 if no element at index.
 */
Ricemotor* ricemotorVectorGet(ricemotorVector* vect, int index) {
	Ricemotor* return_elem;
	if(index < vect->elem_current && index >= 0) {
		return_elem = vect->data[index];
	} else {
		printf("Index not in vector\n\r");
		exit(EXIT_FAILURE);
	}
	return return_elem;
}

/**
 * Initializes a vector
 */
ricepidVector* initRicepidVector() {
	ricepidVector* vect = malloc(sizeof(vect->elem_current) + sizeof(vect->elem_total) + 10*(sizeof(Ricepid*)));
	vect->elem_total = 10;
	vect->elem_current = 0;

	return vect;
}

/**
 * Adds an element to the vector
 *
 * @param vect A pointer to the destination vector
 * @param element A pointer to the new element
 *
 * @return 1 if successful and 0 otherwise
 */
int ricepidVectorAppend(ricepidVector* vect, Ricepid* element) {
	vect->data[vect->elem_current] = element;
	vect->elem_current++;
	if(vect->elem_current >= vect->elem_total) {
		Ricepid* new_data = realloc(vect->data, (vect->elem_total * 2) * sizeof(Ricepid));
		if(new_data) {
			*(vect->data) = new_data;
			vect->elem_total *= 2;
		} else {
			printf("Error allocating memory");
			free(vect->data);
			return 0;
		}
	}
	return 1;
}

/**
 * Returns the element at a given index
 *
 * @param vect A pointer to the destination vector
 * @param index The index of the element to retrieve
 *
 * @return -1 if no element at index.
 */
Ricepid* ricepidVectorGet(ricepidVector* vect, int index) {
	Ricepid* return_elem;
	if(index < vect->elem_current && index >= 0) {
		return_elem = vect->data[index];
	} else {
		printf("Index not in vector");
		exit(EXIT_FAILURE);
	}
	return return_elem;
}

/**
 * Initializes a vector
 */
ricencoderVector* initRicencoderVector() {
	ricencoderVector* vect = malloc(sizeof(vect->elem_current) + sizeof(vect->elem_total) + 10*(sizeof(Ricencoder*)));
	vect->elem_total = 10;
	vect->elem_current = 0;

	return vect;
}

/**
 * Adds an element to the vector
 *
 * @param vect A pointer to the destination vector
 * @param element A pointer to the new element
 *
 * @return 1 if successful and 0 otherwise
 */
int ricencoderVectorAppend(ricencoderVector* vect, Ricencoder* element) {
	vect->data[vect->elem_current] = element;
	vect->elem_current++;
	if(vect->elem_current >= vect->elem_total) {
		Ricencoder* new_data = realloc(vect->data, (vect->elem_total * 2) * sizeof(Ricencoder));
		if(new_data) {
			*(vect->data) = new_data;
			vect->elem_total *= 2;
		} else {
			printf("Error allocating memory");
			free(vect->data);
			return 0;
		}
	}
	return 1;
}

/**
 * Returns the element at a given index
 *
 * @param vect A pointer to the destination vector
 * @param index The index of the element to retrieve
 *
 * @return -1 if no element at index.
 */
Ricencoder* ricencoderVectorGet(ricencoderVector* vect, int index) {
	Ricencoder* return_elem;
	if(index < vect->elem_current && index >= 0) {
		return_elem = vect->data[index];
	} else {
		printf("Index not in vector");
		exit(EXIT_FAILURE);
	}
	return return_elem;
}

/**
 * Initializes a vector
 */
ricepotVector* initRicepotVector() {
	ricepotVector* vect = malloc(sizeof(vect->elem_current) + sizeof(vect->elem_total) + 10*(sizeof(Ricepot*)));
	vect->elem_total = 10;
	vect->elem_current = 0;

	return vect;
}

/**
 * Adds an element to the vector
 *
 * @param vect A pointer to the destination vector
 * @param element A pointer to the new element
 *
 * @return 1 if successful and 0 otherwise
 */
int ricepotVectorAppend(ricepotVector* vect, Ricepot* element) {
	vect->data[vect->elem_current] = element;
	vect->elem_current++;
	if(vect->elem_current >= vect->elem_total) {
		Ricepot* new_data = realloc(vect->data, (vect->elem_total * 2) * sizeof(Ricepot));
		if(new_data) {
			*(vect->data) = new_data;
			vect->elem_total *= 2;
		} else {
			printf("Error allocating memory");
			free(vect->data);
			return 0;
		}
	}
	return 1;
}

/**
 * Returns the element at a given index
 *
 * @param vect A pointer to the destination vector
 * @param index The index of the element to retrieve
 *
 * @return -1 if no element at index.
 */
Ricepot* ricepotVectorGet(ricepotVector* vect, int index) {
	Ricepot* return_elem;
	if(index < vect->elem_current && index >= 0) {
		return_elem = vect->data[index];
	} else {
		printf("Index not in vector");
		exit(EXIT_FAILURE);
	}
	return return_elem;
}

/**
 * Initializes a vector
 */
ricesolenoidVector* initRicesolenoidVector() {
	ricesolenoidVector* vect = malloc(sizeof(vect->elem_current) + sizeof(vect->elem_total) + 10*(sizeof(Ricesolenoid*)));
	vect->elem_total = 10;
	vect->elem_current = 0;

	return vect;
}

/**
 * Adds an element to the vector
 *
 * @param vect A pointer to the destination vector
 * @param element A pointer to the new element
 *
 * @return 1 if successful and 0 otherwise
 */
int ricesolenoidVectorAppend(ricesolenoidVector* vect, Ricesolenoid* element) {
	vect->data[vect->elem_current] = element;
	vect->elem_current++;
	if(vect->elem_current >= vect->elem_total) {
		Ricesolenoid* new_data = realloc(vect->data, (vect->elem_total * 2) * sizeof(Ricesolenoid));
		if(new_data) {
			*(vect->data) = new_data;
			vect->elem_total *= 2;
		} else {
			printf("Error allocating memory");
			free(vect->data);
			return 0;
		}
	}
	return 1;
}

/**
 * Returns the element at a given index
 *
 * @param vect A pointer to the destination vector
 * @param index The index of the element to retrieve
 *
 * @return -1 if no element at index.
 */
Ricesolenoid* ricesolenoidVectorGet(ricesolenoidVector* vect, int index) {
	Ricesolenoid* return_elem;
	if(index < vect->elem_current && index >= 0) {
		return_elem = vect->data[index];
	} else {
		printf("Index not in vector");
		exit(EXIT_FAILURE);
	}
	return return_elem;
}

/**
 * Initializes a vector
 */
ricebuttonVector* initRicebuttonVector() {
	ricebuttonVector* vect = malloc(sizeof(vect->elem_current) + sizeof(vect->elem_total) + 10*(sizeof(Ricebutton*)));
	vect->elem_total = 10;
	vect->elem_current = 0;

	return vect;
}

/**
 * Adds an element to the vector
 *
 * @param vect A pointer to the destination vector
 * @param element A pointer to the new element
 *
 * @return 1 if successful and 0 otherwise
 */
int ricebuttonVectorAppend(ricebuttonVector* vect, Ricebutton* element) {
	vect->data[vect->elem_current] = element;
	vect->elem_current++;
	if(vect->elem_current >= vect->elem_total) {
		Ricebutton* new_data = realloc(vect->data, (vect->elem_total * 2) * sizeof(Ricebutton));
		if(new_data) {
			*(vect->data) = new_data;
			vect->elem_total *= 2;
		} else {
			printf("Error allocating memory");
			free(vect->data);
			return 0;
		}
	}
	return 1;
}

/**
 * Returns the element at a given index
 *
 * @param vect A pointer to the destination vector
 * @param index The index of the element to retrieve
 *
 * @return -1 if no element at index.
 */
Ricebutton* ricebuttonVectorGet(ricebuttonVector* vect, int index) {
	Ricebutton* return_elem;
	if(index < vect->elem_current && index >= 0) {
		return_elem = vect->data[index];
	} else {
		printf("Index not in vector");
		exit(EXIT_FAILURE);
	}
	return return_elem;
}

// A typical init function
/*
void initialize() {
	riceBotInitialize();

	driveTrainStyle = DTFOURWHEELS;
	controlStyle = CTCHEEZYDRIVE;

	MOTDTFrontLeft = initRicemotor(9, 1);
	MOTDTFrontRight = initRicemotor(8, -1);
	MOTDTBackLeft = initRicemotor(3, 1);
	MOTDTBackRight = initRicemotor(2, -1);

	MOTARMLeft = initRicemotor(6, -1);
	MOTARMRight = initRicemotor(5, -1);

	MOTARMFront = initRicemotor(7, 1);
	MOTConveyor = initRicemotor(4, 1);
	MOTCLAW = initRicemotor(10, -1);
	MOTCOL = initRicemotor(9, 1);

	EncDTLeft = initRicencoderIME(627.2, 1, 0, false);
	EncDTRight = initRicencoderIME(627.2, 1, 1, true);

	EncARMLeft = initRicencoderIME(627.2, 1, 3, true);
	EncARMRight = initRicencoderIME(627.2, 1, 2, false);
	EncARMFront = initRicencoderIME(627.2, 1, 4, false);

//	ButConLeft = initRicebutton(2);
//	ButConRight = initRicebutton(1);
	ButARMBase = initRicebutton(4);
//	ButARMFrontLeft = initRicebutton(5);
//	ButARMFrontRight = initRicebutton(6);

	gyro = initRicegyro(1, 196);

	Ricemotor* armLeft[2] = {MOTARMLeft, MOTDefault};
	Ricemotor* armRight[2] = {MOTARMRight, MOTDefault};
	PidARMLeft = initRicepid(&(EncARMLeft->adjustedValue), .2, 0, 0, armLeft);
	PidARMRight = initRicepid(&(EncARMRight->adjustedValue), .2, 0, 0, armRight);
	PidARMLeft->running = 1;
	PidARMRight->running = 1;

	delay(500);

	taskCreate(IOTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_HIGHEST);
	taskCreate(PidTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
	taskCreate(miscTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
}*/

// A typical miscTask
/*
void miscTask(void *ignore) {
	while(1) {

//		printf("EncARMFront: %d\n\r", EncARMFront->adjustedValue);
		printf("Buttons: %d|%d|%d|%d|%d\n\r", ButConLeft->state, ButConRight->state, ButARMBase->state,
				ButARMFrontLeft->state, ButARMFrontRight->state);

//		printf("DriveTrain: %d|%d | %d|%d\n\r", MOTDTFrontLeft->out, MOTDTFrontRight->out,
//				MOTDTBackLeft->out, MOTDTBackRight->out);
//
//		printf("Setpoint: %d|%d, Raw: %d|%d, Adj: %d|%d, Out: %d|%d, Pid: %d|%d\n\r",
//				PidARMLeft->setPoint, PidARMRight->setPoint,
//				EncARMLeft->rawValue, EncARMRight->rawValue,
//				EncARMLeft->adjustedValue, EncARMRight->adjustedValue,
//				MOTARMLeft->out, MOTARMRight->out,
//				PidARMLeft->running, PidARMRight->running);
//
//		printf("Gyro: %d\n\r", gyro->value);
//
//		printf("Power: %dmV\n\r", powerLevelMain());
		delay(20);
	}
}
 */
