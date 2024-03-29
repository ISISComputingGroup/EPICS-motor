# motorIms
EPICS motor drivers for the following [Schneider Electric, formerly IMS](https://motion.schneider-electric.com) controllers: IM483, MDrive and MForce

[![Build Status](https://github.com/epics-motor/motorIms/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorIms/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorIms.png)](https://travis-ci.org/epics-motor/motorIms)-->

motorIms is a submodule of [motor](https://github.com/epics-modules/motor).  When motorIms is built in the ``motor/modules`` directory, no manual configuration is needed.

motorIms can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorIms contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
