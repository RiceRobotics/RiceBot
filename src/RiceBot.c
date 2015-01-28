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

#include "main.h"

/*
 * Initializes a Motor type
 *
 * @param port The port on the Cortex which the motor is plugged into
 * @param reflected If the output to the motor should be reversed. -1 or 1
 *
 * @return The initialized and configured motor
 */
Motor initMotor(unsigned char port, int reflected) {
	Motor *m = malloc(sizeof(Motor));
	m->port = port;
	m->out = 0;
	m->reflected = reflected;
	return *m;
}

/*
 * Initializes a Pid type
 *
 * @param kP The coefficient for the proportional term
 * @param kI The coefficient for the integral term
 * @param kD The coefficient for the derivative term
 *
 * @return The initialized and configured Pid
 */
Pid initPid(float kP, float kI, float kD) {
	Pid *p = malloc(sizeof(Pid));
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
	return *p;
}

/*
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
 * @param reverse (If not IME) Whether the QuadEncoder should count in the reverse direction
 *
 * @return The initialized and configured Ricencoder
 */
Ricencoder initRicencoder(float ticksPerRev, int mult, int isIME, unsigned char imeAddress,
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
	if(!isIME) {
		r->enc = encoderInit(portTop, portBot, reverse);
	}
	return *r;
}

/*
 * Initializes a Ricepot
 *
 * @param port The port on the Cortex which the potentiometer is plugged into
 *
 * @return The initialized and configured Ricepot
 */
Ricepot initRicepot(unsigned char port) {
	analogCalibrate(port);
	Ricepot *r = malloc(sizeof(Ricepot));
	r->port = port;
	r->value = 0;
	return *r;
}

/*
 * Initializes a Ricegyro
 *
 * @param port The port on the Cortex which the gyro is plugged into
 * @param multiplier Sensitivity calibration for the gyro. Use 0 for the default value of 196
 */
Ricegyro initRicegyro(unsigned char port, unsigned short multiplier) {
	Ricegyro *gyro = malloc(sizeof(Ricegyro));
	gyro->port = port;
	gyro->multiplier = multiplier;
	printf("Ricegyro Initialization (no touchie the robutt\n\r");
	gyro->g = gyroInit(gyro->port, gyro->multiplier);
	delay(1000);
	printf("Ricegyro Initialized with minimal casualties\n\r");
	gyroReset(gyro->g);
	gyro->value = 0;
	return *gyro;
}

void riceBotInitializeIO() {

}

/*
 * Call this from the default Initialize function.
 * After, be sure to reinitialize each motor you will be using on your robot.
 */
void riceBotInitialize() {

//	ENCDTLeft = encoderInit(0, 0, false);
//	ENCDTRight = encoderInit(0, 0, false);
//	ENCARMLeft = encoderInit(0, 0, false);
//	ENCARMRight = encoderInit(0, 0, false);

}

/*
 * Checks joystick input and sets all Motor structs to appropriate output
 * @param controlStyle The format of the joystick input.
 * 			Can be:
 * 		 			TANKDRIVE
 * 	 	 			ARCADEDRIVE
 *	 	 			CHEEZYDRIVE
 *	 	 			MECANUMDRIVE
 *	 	 			HDRIVE
 */
void getJoystickForDriveTrain() {
	int x1 = joystickGetAnalog(1, 4);
	int y1 = joystickGetAnalog(1, 3);
	int x2 = joystickGetAnalog(1, 1);
	int y2 = joystickGetAnalog(1, 2);

	switch(controlStyle) {
	case CTTANKDRIVE:
		MOTDTFrontLeft.out = y1;
		MOTDTFrontMidLeft.out = y1;
		MOTDTMidLeft.out = y1;
		MOTDTBackLeft.out = y1;

		MOTDTFrontRight.out = y2;
		MOTDTFrontMidRight.out = y2;
		MOTDTMidRight.out = y2;
		MOTDTBackRight.out = y2;
		break;
	case CTARCADEDRIVE:
		MOTDTFrontLeft.out = (y1 + x1) / 2;
		MOTDTFrontMidLeft.out = (y1 + x1) / 2;
		MOTDTMidLeft.out = (y1 + x1) / 2;
		MOTDTBackLeft.out = (y1 + x1) / 2;

		MOTDTFrontRight.out = (y1 - x1) / 2;
		MOTDTFrontMidRight.out = (y1 - x1) / 2;
		MOTDTMidRight.out = (y1 - x1) / 2;
		MOTDTBackRight.out = (y1 - x1) / 2;
		break;
	case CTCHEEZYDRIVE:
		MOTDTFrontLeft.out = (y1 + x2) / 2;
		MOTDTFrontMidLeft.out = (y1 + x2) / 2;
		MOTDTMidLeft.out = (y1 + x2) / 2;
		MOTDTBackLeft.out = (y1 + x2) / 2;

		MOTDTFrontRight.out = (y1 - x2) / 2;
		MOTDTFrontMidRight.out = (y1 - x2) / 2;
		MOTDTMidRight.out = (y1 - x2) / 2;
		MOTDTBackRight.out = (y1 - x2) / 2;
		break;
	case CTMECANUMDRIVE:
		MOTDTFrontLeft.out = y1 + x2 + x1;
		MOTDTBackLeft.out = y1 + x2 - x1;

		MOTDTFrontRight.out = y1 - x2 - x1;
		MOTDTBackRight.out = y1 - x2 + x1;
		break;
	case CTHDRIVE:
	default:
		break;
	}
}

/* Final stage: sets all physical motors based on output set in Motor structs
 * Run in a task?
 * @param driveTrainStyle The configuration of the wheels on the robot.
 * 			Can be:
 * 					FOURWHEELS
 * 					SIXWHEELS
 * 					EIGHTWHEELS
 * 					MECANUM
 * 					HOLONOMIC
 * 					HDRIVE
 * 					SWERVE
 */
void setDriveTrainMotors() {
	switch(driveTrainStyle) {
	case DTFOURWHEELS:
		motorSet(MOTDTFrontLeft.port, MOTDTFrontLeft.out * MOTDTFrontLeft.reflected);
		motorSet(MOTDTBackLeft.port, MOTDTBackLeft.out * MOTDTBackLeft.reflected);

		motorSet(MOTDTFrontRight.port, MOTDTFrontRight.out * MOTDTFrontRight.reflected);
		motorSet(MOTDTBackRight.port, MOTDTBackRight.out * MOTDTBackRight.reflected);
		break;
	case DTSIXWHEELS:
		motorSet(MOTDTFrontLeft.port, MOTDTFrontLeft.out * MOTDTFrontLeft.reflected);
		motorSet(MOTDTMidLeft.port, MOTDTMidLeft.out * MOTDTMidLeft.reflected);
		motorSet(MOTDTBackLeft.port, MOTDTBackLeft.out * MOTDTBackLeft.reflected);

		motorSet(MOTDTFrontRight.port, MOTDTFrontRight.out * MOTDTFrontRight.reflected);
		motorSet(MOTDTMidRight.port, MOTDTMidRight.out * MOTDTMidRight.reflected);
		motorSet(MOTDTBackRight.port, MOTDTBackRight.out * MOTDTBackRight.reflected);
		break;
	case DTEIGHTWHEELS:
		motorSet(MOTDTFrontLeft.port, MOTDTFrontLeft.out * MOTDTFrontLeft.reflected);
		motorSet(MOTDTFrontMidLeft.port, MOTDTFrontMidLeft.out * MOTDTFrontMidLeft.reflected);
		motorSet(MOTDTMidLeft.port, MOTDTMidLeft.out * MOTDTMidLeft.reflected);
		motorSet(MOTDTBackLeft.port, MOTDTBackLeft.out * MOTDTBackLeft.reflected);

		motorSet(MOTDTFrontRight.port, MOTDTFrontRight.out * MOTDTFrontRight.reflected);
		motorSet(MOTDTFrontMidRight.port, MOTDTFrontMidRight.out * MOTDTFrontMidRight.reflected);
		motorSet(MOTDTMidRight.port, MOTDTMidRight.out * MOTDTMidRight.reflected);
		motorSet(MOTDTBackRight.port, MOTDTBackRight.out * MOTDTBackRight.reflected);
		break;
	default:
		break;
	}
}

/*
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
	rc->adjustedValue = rc->rawValue * rc->mult;
}

/*
 * Updates the value of any Ricegyro based on a pointer to the struct
 *
 * @param *rg A pointer to the Ricegyro struct
 */
void updateRicegyro(Ricegyro *rg) {
	rg->value = gyroGet(rg->g);
}

void autonomousTask(int instruction, int distance, int pow, long timeout) {
	int target;
	long startTime = millis();

	int power[2];
	power[1] = (pow == NULL) ? 127 : pow;
	power[0] = power[1];

	switch(instruction) {
	case AUTODRIVETIME:
		while(millis() < startTime + timeout) {
			MOTDTFrontRight.out = power[1];
			MOTDTFrontMidRight.out = power[1];
			MOTDTMidRight.out = power[1];
			MOTDTBackRight.out = power[1];
			MOTDTFrontLeft.out = power[0];
			MOTDTFrontMidLeft.out = power[0];
			MOTDTMidLeft.out = power[0];
			MOTDTBackLeft.out = power[0];
		}
		break;
	case AUTODRIVEBASIC:
		target = EncDTLeft.ticksPerRev / (4 * MATH_PI) * distance;
		//		power = (pow == NULL) ? 127 : pow;
		int current[2] = {EncDTLeft.rawValue, EncDTRight.rawValue};

		while(current[1] < target && millis() < startTime + timeout) {
			if(abs(current[1] - current[0]) > 50) {
				if(current[0] > current[1]) {
					power[0] = speedRegulator(power[0] - 2);
				} else if(current[0] < current[1]) {
					power[0] = speedRegulator(power[0] + 2);
				}
			}

			MOTDTFrontRight.out = power[1];
			MOTDTFrontMidRight.out = power[1];
			MOTDTMidRight.out = power[1];
			MOTDTBackRight.out = power[1];
			MOTDTFrontLeft.out = power[0];
			MOTDTFrontMidLeft.out = power[0];
			MOTDTMidLeft.out = power[0];
			MOTDTBackLeft.out = power[0];

			delay(20);
			current[0] = EncDTLeft.rawValue;
			current[1] = EncDTRight.rawValue;
		}
		break;
	case AUTOTURNBASIC:
		target = distance;
		if(target < gyro.value) {		//Left Turn
			while(gyro.value > target && millis() < startTime + timeout) {
				MOTDTFrontRight.out = pow;
				MOTDTFrontMidRight.out = pow;
				MOTDTMidRight.out = pow;
				MOTDTBackRight.out = pow;
				MOTDTFrontLeft.out = -pow;
				MOTDTFrontMidLeft.out = -pow;
				MOTDTMidLeft.out = -pow;
				MOTDTBackLeft.out = -pow;
			}
		}
		else if(target > gyro.value) {	//Right Turn
			while(gyro.value < target && millis() < startTime + timeout) {
				MOTDTFrontRight.out = -pow;
				MOTDTFrontMidRight.out = -pow;
				MOTDTMidRight.out = -pow;
				MOTDTBackRight.out = -pow;
				MOTDTFrontLeft.out = pow;
				MOTDTFrontMidLeft.out = pow;
				MOTDTMidLeft.out = pow;
				MOTDTBackLeft.out = pow;
			}
		}
		break;
	case AUTODRIVEGYRO:
		target = EncDTLeft.ticksPerRev / (4 * MATH_PI) * distance;
		int targetGyro = gyro.value;


		break;
	case AUTOCOLLECTORS:
		if(timeout == NULL) {
			MOTCOL.out = pow;
			MOTCOLLeft.out = pow;
			MOTCOLRight.out = pow;
		}
		else {
			while (millis() < startTime + timeout) {
				MOTCOL.out = pow;
				MOTCOLLeft.out = pow;
				MOTCOLRight.out = pow;
			}
			MOTCOL.out = 0;
			MOTCOLLeft.out = 0;
			MOTCOLRight.out = 0;
		}
		break;
	}
}

/*
 * Updates a PID loop based on the value of a sensor
 *
 * @param *pidLoop A pointer to the pid struct to update
 * @param current The current value of the relevant sensor to use with the pid loop
 *
 */
void processPid(Pid *pidLoop, int current) {
	if (pidLoop->running) {
		pidLoop->current = current;
		//	printf("Current: %d\n\r", pidLoop->current);
		pidLoop->error = pidLoop->setPoint - pidLoop->current;
		pidLoop->integral += pidLoop->error;
		pidLoop->derivative = pidLoop->lastError - pidLoop->error;

		pidLoop->output = speedRegulator((pidLoop->error * pidLoop->kP) +
				(pidLoop->integral * pidLoop->kI) + (pidLoop->derivative * pidLoop->kD));

		pidLoop->lastError = pidLoop->error;
	}

}

/*
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

/*
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

/*
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
