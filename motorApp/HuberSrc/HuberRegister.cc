/*
FILENAME...	HuberRegister.cc
USAGE...	Register Huber motor device driver shell commands.

Version:	$Revision: 1.4 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-07-16 19:06:58 $
*/

/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/

#include <iocsh.h>
#include "HuberRegister.h"
#include "epicsExport.h"

extern "C"
{

// Huber Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Huber Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};

static const iocshArg * const HuberSetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const HuberConfigArgs[2] = {&configArg0, &configArg1};

static const iocshFuncDef setupHuber  = {"HuberSetup",  2, HuberSetupArgs};
static const iocshFuncDef configHuber = {"HuberConfig", 2, HuberConfigArgs};

} // extern "C"
