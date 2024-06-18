# motorScriptMotor
EPICS motor drivers for Lua ScriptMotor.  Requires the EPICS [lua](https://github.com/epics-modules/lua) module.

[![Build Status](https://github.com/epics-motor/motorScriptMotor/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorScriptMotor/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorScriptMotor.png)](https://travis-ci.org/epics-motor/motorScriptMotor)-->

motorScriptMotor is a submodule of [motor](https://github.com/epics-modules/motor).  When motorScriptMotor is built in the ``motor/modules`` directory, no manual configuration is needed.

motorScriptMotor can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorScriptMotor contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
