# motorMXmotor
EPICS motor support for interfacing the [MX control system](http://mx.iit.edu/)

[![Build Status](https://github.com/epics-motor/motorMXmotor/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorMXmotor/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorMXmotor.png)](https://travis-ci.org/epics-motor/motorMXmotor)-->

motorMXmotor is a submodule of [motor](https://github.com/epics-modules/motor).  When motorMXmotor is built in the ``motor/modules`` directory, no manual configuration is needed.

motorMXmotor can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorMXmotor contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.

motorMXmotor requires the [MX control system](http://mx.iit.edu/).
