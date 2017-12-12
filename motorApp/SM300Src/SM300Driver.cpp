/*
FILENAME... SM300Driver.cpp
USAGE...    Motor driver support for the Newport SM300 controller.

Based on the ACS MCB-4B Model 3 device driver written by:
Mark Rivers
March 1, 2012

K. Goetze 2012-03-23  Initial version
          2013-06-07  Allow motor resolution to be set using "SM300CreateController" at boot time

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
#include "SM300Driver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new SM300Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SM300PortName     The name of the drvAsynSerialPort that was created previously to connect to the SM300 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SM300Controller::SM300Controller(const char *portName, const char *SM300PortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod, double stepSize)
  :  asynMotorController(portName, numAxes, NUM_SM300_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  SM300Axis *pAxis;
  static const char *functionName = "SM300Controller::SM300Controller";
  
  /* Connect to SM300 controller */
  status = pasynOctetSyncIO->connect(SM300PortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to SM300 controller\n",
      functionName);
  }
  for (axis=0; axis<numAxes; axis++) {
  //for (axis=1; axis < (numAxes + 1); axis++) {
    pAxis = new SM300Axis(this, axis, stepSize);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new SM300Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SM300PortName       The name of the drvAsynIPPPort that was created previously to connect to the SM300 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  * \param[in] eguPerStep        The stage resolution
  */
extern "C" int SM300CreateController(const char *portName, const char *SM300PortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod, const char *eguPerStep)
{
  double stepSize;
   
  stepSize = strtod(eguPerStep, NULL);
  new SM300Controller(portName, SM300PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000., stepSize);
  //printf("\n *** SM300: stepSize=%f\n", stepSize);
  if (errno != 0) {
    printf("SM300: Error invalid steps per unit=%s\n", eguPerStep);
    return asynError;
  }
  
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void SM300Controller::report(FILE *fp, int level)
{
  fprintf(fp, "SM300 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an SM300Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
SM300Axis* SM300Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<SM300Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SM300Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] No Axis index number. */
SM300Axis* SM300Controller::getAxis(int axisNo)
{
  return static_cast<SM300Axis*>(asynMotorController::getAxis(axisNo));
}











/** Polls the axis.
* This function reads motor position, limit status, home status, and moving status
* It calls setIntegerParam() and setDoubleParam() for each item that it polls,
* and then calls callParamCallbacks() at the end.
* \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus SM300Controller::poll()
{
	int done;
	//int driveOn;
	int limit;
	double position;
	asynStatus comStatus;
	char * position_start;
	SM300Axis *axis;
	// Read the current motor position
	sprintf(this->outString_, "LQ");
		

	comStatus = this->writeReadController();
	if (comStatus) goto skip;
	
	// The response string is of the form "\06\02X%d,Y%d"
	position_start = &this->inString_[3];
	position = (atof(position_start) / 1.0);
	//printf("Position0: %f", position);
	axis = this->getAxis(0);
	axis->setDoubleParam(this->motorPosition_, position);
	axis->callParamCallbacks();
	axis->setIntegerParam(this->motorStatusDone_, 0);
	position_start = strchr(this->inString_, ',');

	position = (atof(&position_start[2]) / 1.0);
	//printf("Position1: %f", position);
	axis = this->getAxis(1);
	axis->setDoubleParam(this->motorPosition_, position);
	axis->setIntegerParam(this->motorStatusDone_, 1);
	axis->callParamCallbacks();

	// Read the moving status of this motor
//	sprintf(this->outString_, "%1dTS", axisNo_ + 1);
//	comStatus = this->writeReadController();
//	if (comStatus) goto skip;
	// The response string is of the form "1TS000028"
	// May need to add logic for moving while homing
//	done = ((this->inString_[7] == '2') && (this->inString_[8] == '8')) ? 0 : 1;
	

	// Read the limit status
	// The response string is of the form "1TS001328"
	//
	//   The stage I tested this with is a GTS30V vertical jack.  When the controller is initialized
	//   for this device using Newport's software, +25 and -5 limits get set.  The controller does not
	//   let you set limits outside these values, so I never was able to run into a "hard" limit. 
	//   The controller also does not allow position settings outside these limits, and does not give
	//   an indication.  So, my recommendation is to leave the controller's travel limits set to the
	//   Newport defaults, and use the motor record's soft limits, set to the controller's limits or within.
	//
	// Should a hard limit be actually encounted, this code *should* report it to the motor record
//	limit = (this->inString_[6] == '2') ? 1 : 0;
//	setIntegerParam(this->motorStatusHighLimit_, limit);
//	limit = (this->inString_[6] == '1') ? 1 : 0;
//	setIntegerParam(this->motorStatusLowLimit_, limit);
//	limit = ((this->inString_[7] == '3') && (this->inString_[8] == '2')) ? 1 : 0;
//	setIntegerParam(this->motorStatusAtHome_, limit);

	// Read the drive power on status
	//sprintf(this->outString_, "#%02dE", axisNo_ + 1);
	//comStatus = this->writeReadController();
	//if (comStatus) goto skip;
	//driveOn = (this->inString_[5] == '1') ? 1:0;
	//setIntegerParam(this->motorStatusPowerOn_, driveOn);
	//setIntegerParam(this->motorStatusProblem_, 0);

skip:
	axis->setIntegerParam(this->motorStatusProblem_, comStatus ? 1 : 0);
	callParamCallbacks();
	return comStatus ? asynError : asynSuccess;
}






// These are the SM300Axis methods

/** Creates a new SM300Axis object.
  * \param[in] pC Pointer to the SM300Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
SM300Axis::SM300Axis(SM300Controller *pC, int axisNo, double stepSize)
  : asynMotorAxis(pC, axisNo),
    pC_(pC), stepSize_(stepSize)
{ 

}



/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void SM300Axis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_ + 1);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus SM300Axis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  // static const char *functionName = "SM300::sendAccelAndVelocity";

  // Send the velocity in egus
  sprintf(pC_->outString_, "%1dVA%f", axisNo_ + 1, (velocity*stepSize_));
  status = pC_->writeController();

  // Send the acceleration in egus/sec/sec
  //printf("velocity: %f\n", velocity);
  //printf("acceleration: %f", acceleration);
  sprintf(pC_->outString_, "%1dAC%f", axisNo_ + 1, (acceleration*stepSize_));
  status = pC_->writeController();
  return status;
}


asynStatus SM300Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "SM300Axis::move";

  status = sendAccelAndVelocity(acceleration, maxVelocity);
  
  if (relative) {
    sprintf(pC_->outString_, "%1dPR%f", axisNo_ + 1, (position*stepSize_));
  } else {
    sprintf(pC_->outString_, "%1dPA%f", axisNo_ + 1, (position*stepSize_));
  }
  status = pC_->writeController();
  return status;
}

asynStatus SM300Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "SM300Axis::home";

  // Must be in unreferenced state to home, so can only home once after a reset
  // this code should force a reset and allow a rehome, but controller doesn't seem happy
  //  sprintf(pC_->outString_, "%1dRS", axisNo_ + 1);
  //  status = pC_->writeController();
  //  epicsThreadSleep(5.0);
  
  // set Home search velocity
  //sprintf(pC_->outString_, "%1dOH%f", axisNo_ + 1, maxVelocity);
  //status = pC_->writeController();

  sprintf(pC_->outString_, "%1dOR", axisNo_ + 1);
  
  status = pC_->writeController();
  return status;
}

// Jog
asynStatus SM300Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  double high_limit;
  double low_limit;
  asynStatus comStatus;
  static const char *functionName = "SM300Axis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
    functionName, minVelocity, maxVelocity, acceleration);
    
  comStatus = sendAccelAndVelocity(acceleration, maxVelocity);
  if (comStatus) goto skip;

  /* SM300 supports the notion of jog, but only for a remote control keypad */
  // SM300 will not allow moves outside of those set with the SL and SR commands
  // first we query these limits and then make the jog a move to the limit
  
  // get the high limit
  sprintf(pC_->outString_, "%1dSR?", axisNo_ + 1);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "1SR25.0"
  high_limit = (atof(&pC_->inString_[3]));
  
    // get the low limit
  sprintf(pC_->outString_, "%1dSL?", axisNo_ + 1);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "1SL-5.0"
  low_limit = (atof(&pC_->inString_[3]));
  
  if (maxVelocity > 0.) {
    /* This is a positive move in SM300 coordinates (egus) */
    sprintf(pC_->outString_, "%1dPA%f", axisNo_ + 1, high_limit);
  } else {
      /* This is a negative move in SM300 coordinates (egus) */
      sprintf(pC_->outString_, "%1dPA%f", axisNo_ + 1, low_limit);
  }
  comStatus = pC_->writeController();
  if (comStatus) goto skip;
  
  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;

}

asynStatus SM300Axis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "SM300Axis::stop";

  sprintf(pC_->outString_, "%1dST", axisNo_ + 1);
  status = pC_->writeController();
  return status;
}

asynStatus SM300Axis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "SM300Axis::setPosition";

  // ? not sure yet
  //sprintf(pC_->outString_, "#%02dP=%+d", axisNo_ + 1, NINT(position));
  status = pC_->writeReadController();
  return status;
}

asynStatus SM300Axis::setClosedLoop(bool closedLoop)
{
  asynStatus status;
  //static const char *functionName = "SM300Axis::setClosedLoop";

  // ? not sure yet
  //sprintf(pC_->outString_, "#%02dW=%d", axisNo_ + 1, closedLoop ? 1:0);
  status = pC_->writeReadController();
  return status;
}



/** Code for iocsh registration */
static const iocshArg SM300CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SM300CreateControllerArg1 = {"SM300 port name", iocshArgString};
static const iocshArg SM300CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg SM300CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SM300CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg SM300CreateControllerArg5 = {"EGUs per step", iocshArgString};
static const iocshArg * const SM300CreateControllerArgs[] = {&SM300CreateControllerArg0,
                                                             &SM300CreateControllerArg1,
                                                             &SM300CreateControllerArg2,
                                                             &SM300CreateControllerArg3,
                                                             &SM300CreateControllerArg4,
															 &SM300CreateControllerArg5};
static const iocshFuncDef SM300CreateControllerDef = {"SM300CreateController", 6, SM300CreateControllerArgs};
static void SM300CreateContollerCallFunc(const iocshArgBuf *args)
{
  SM300CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval);
}

static void SM300Register(void)
{
  iocshRegister(&SM300CreateControllerDef, SM300CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(SM300Register);
}
