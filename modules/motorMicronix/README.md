# motorMicronix
EPICS motor drivers for the following [Micronix](http://www.micronixusa.com) controllers: MMC-100 and MMC-200

[![Build Status](https://github.com/epics-motor/motorMicronix/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorMicronix/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorMicronix.png)](https://travis-ci.org/epics-motor/motorMicronix)-->

motorMicronix is a submodule of [motor](https://github.com/epics-modules/motor).  When motorMicronix is built in the ``motor/modules`` directory, no manual configuration is needed.

motorMicronix can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorMicronix contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
