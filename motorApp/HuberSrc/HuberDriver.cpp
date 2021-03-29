/*
Motor driver support for the Huber controller.

Based on the SM100 Model 3 device driver
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include <errlog.h>
#include "HuberDriver.h"

/** Creates a new HuberController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] HuberPortName     The name of the drvAsynSerialPort that was created previously to connect to the Huber controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
HuberController::HuberController(const char *portName, const char *HuberPortName, int numAxes,
	double movingPollPeriod, double idlePollPeriod)
	: asynMotorController(portName, numAxes, NUM_Huber_PARAMS,
		0, // No additional interfaces beyond those in base class
		0, // No additional callback interfaces beyond those in base class
		ASYN_CANBLOCK | ASYN_MULTIDEVICE,
		1, // autoconnect
		0, 0),  // Default priority and stack size
	is_moving_(false),
	polls_count_(0)
{
	//setTerminationChars(";\r\n", 1, "\r", 1);
	asynStatus status;
	static const char *functionName = "HuberController::HuberController";

	/* Connect to Huber controller */
	status = pasynOctetSyncIO->connect(HuberPortName, 0, &pasynUserController_, NULL);
	if (status) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
			"%s: cannot connect to Huber controller\n",
			functionName);
	}
	if (numAxes != 2) {
		errlogPrintf("Huber: Driver is only setup for two axes X and Y!\n");
	}
	new HuberAxis(this, 0, 'A');
	new HuberAxis(this, 1, 'B');
	new HuberAxis(this, 2, 'C');
	new HuberAxis(this, 3, 'D');
	new HuberAxis(this, 4, 'E');
	new HuberAxis(this, 5, 'F');
	new HuberAxis(this, 6, 'G');
	new HuberAxis(this, 7, 'H');

	createParam(HuberResetString, asynParamInt32, &reset_);
	createParam(HuberResetAndHomeString, asynParamInt32, &reset_and_home_);
	createParam(HuberDisconnectString, asynParamInt32, &disconnect_);
	createParam(HuberErrorCodeString, asynParamInt32, &error_code_);

	startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Send a query string to the controller and get a return value.
  * query string is prefeced with ACK STX and postfixed with EOT
  * return string ends with ETX
  * \param[in] query the query to send
  * \param[in] what form the query reply has; true if the reply has EOT false for ETX with BCC
  * \returns success of write and read of query string
  */
asynStatus HuberController::sendQuery(const char * query, bool hasEotEnding) {
	if (hasEotEnding) {
		setTerminationChars("\x04", 1, "\x04", 1);
	}
	else {
		setTerminationChars("\x03", 1, "\x04", 1);
	}
	//send data format 2
	sprintf(this->outString_, "\x06\x02%s", query);
	return this->writeReadController();
}

/** Send a command string to the controller.
* Command string is prefeced with ACK STX and postfixed with EOT
* \param[in] command the command to send
* \returns success of write and read of acknowledgement from controller
*/
asynStatus HuberController::sendCommand(const char * command) {
	setTerminationChars("\r", 1, "\r\n", 1);
	//send data format 2
	sprintf(this->outString_, "%s", command);
	printf("\nFull cmd string: %s", this->outString_);
	return this->writeReadController();
}

/**
  * Return true if the controller registering moving motors
  */
bool HuberController::is_moving() {
	return is_moving_;
}

/**
  * deal with db records being set which are integers
  * \returns status
  */
asynStatus HuberController::writeInt32(asynUser *pasynUser, epicsInt32 value) {
	asynStatus status;
	int function = pasynUser->reason;		//Function requested
	if (function == reset_) {
		if (value == 0) return asynSuccess;
		callParamCallbacks();
	}
	else if (function == disconnect_) {
		setIntegerParam(disconnect_, 1);
		setTerminationChars("\x06", 1, "\x04", 1);
		sprintf(this->outString_, "\x06\x02%s", "M77");
		status = writeController();
		setIntegerParam(disconnect_, 0);
		callParamCallbacks();
	}
	else if (function == reset_and_home_) {
		setIntegerParam(reset_and_home_, 1);
		for (int i = 0; i < numAxes_; i++) {
			getAxis(i)->home(0, 0, 0, 0);
		}
		setIntegerParam(reset_and_home_, 0);
		callParamCallbacks();
	}
	else {
		status = asynMotorController::writeInt32(pasynUser, value);
	}
	return status;
}


/** Creates a new HuberController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] HuberPortName       The name of the drvAsynIPPPort that was created previously to connect to the Huber controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  * \param[in] eguPerStep        The stage resolution
  */
extern "C" int HuberCreateController(const char *portName, const char *HuberPortName, int numAxes,
	int movingPollPeriod, int idlePollPeriod)
{
	new HuberController(portName, HuberPortName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.);
	//printf("\n *** Huber: stepSize=%f\n", stepSize);

	return(asynSuccess);
}

/** Returns a pointer to an HuberAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
HuberAxis* HuberController::getAxis(asynUser *pasynUser)
{
	return static_cast<HuberAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an HuberAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] No Axis index number. */
HuberAxis* HuberController::getAxis(int axisNo)
{
	return static_cast<HuberAxis*>(asynMotorController::getAxis(axisNo));
}

/** Set the termination characters on the output and input buffers
 * \param[in] eosIn end of string for input
 * \param[in] eosInlen length of end of string for input
 * \param[in] eosOut end of string for output
 * \param[in] eosOutlen length of end of string for output
 */
void HuberController::setTerminationChars(const char *eosIn, int eosInlen, const char *eosOut, int eosOutlen) {

	pasynOctetSyncIO->setOutputEos(this->pasynUserController_, eosOut, eosOutlen);
	pasynOctetSyncIO->setInputEos(this->pasynUserController_, eosIn, eosInlen);
}

// These are the HuberAxis methods

/** Creates a new HuberAxis object.
  * \param[in] pC Pointer to the HuberController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
HuberAxis::HuberAxis(HuberController *pC, int axisNo, char axisLabel)
	: asynMotorAxis(pC, axisNo),
	pC_(pC), axisLabel(axisLabel)
{
	this->axisNo = axisNo;
}

/** Get an whether the axis has an error.
*/
bool HuberAxis::has_error() {
	return has_error_;
}

/** Set absolute position in hardware
  * Not supported
  */
asynStatus HuberAxis::setPosition(double position)
{
	char temp[40];
	asynStatus comStatus = pC_->sendCommand(temp);
	return comStatus;
}

/** Move the motor to an absolute location or by a relative amount (uses current location since the motor doesw not support a relative command).
* \param[in] position  The absolute position to move to (if relative=0) or the relative distance to move
* by (if relative=1). Units=steps.
* \param[in] relative  Flag indicating relative move (1) or absolute move (0).
* \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
* \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
* \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus HuberAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
	asynStatus comStatus;
	char temp[MAX_CONTROLLER_STRING_SIZE];
	double move_to = position;
	std::string cmdString  = "goto";

	if (relative == 1) { //relative move
		cmdString = "move";
	}

	sprintf(temp, "%s%d:%.0f", cmdString, this->axisNo, position);
	comStatus = pC_->sendCommand(temp);

	if (comStatus) goto skip;
	comStatus = pC_->sendCommand("BSL");

skip:
	return comStatus;

}
/** Code for iocsh registration */
static const iocshArg HuberCreateControllerArg0 = { "Port name", iocshArgString };
static const iocshArg HuberCreateControllerArg1 = { "Huber port name", iocshArgString };
static const iocshArg HuberCreateControllerArg2 = { "Number of axes", iocshArgInt };
static const iocshArg HuberCreateControllerArg3 = { "Moving poll period (ms)", iocshArgInt };
static const iocshArg HuberCreateControllerArg4 = { "Idle poll period (ms)", iocshArgInt };
static const iocshArg * const HuberCreateControllerArgs[] = { &HuberCreateControllerArg0,
															 &HuberCreateControllerArg1,
															 &HuberCreateControllerArg2,
															 &HuberCreateControllerArg3,
															 &HuberCreateControllerArg4 };
static const iocshFuncDef HuberCreateControllerDef = { "HuberCreateController", 5, HuberCreateControllerArgs };
static void HuberCreateContollerCallFunc(const iocshArgBuf *args)
{
	HuberCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void HuberRegister(void)
{
	iocshRegister(&HuberCreateControllerDef, HuberCreateContollerCallFunc);
}

extern "C" {
	epicsExportRegistrar(HuberRegister);
}
