# motorHytec Releases

## __R1-0-3 (2023-04-11)__
R1-0-3 is a release based on the master branch.

### Changes since R1-0-2

#### New features
* None

#### Modifications to existing features
* None

#### Bug fixes
* None

#### Continuous integration
* Added ci-scripts (v3.0.1)
* Configured to use Github Actions for CI

## __R1-0-2 (2020-05-14)__
R1-0-2 is a release based on the master branch.  

### Changes since R1-0-1

#### New features
* None

#### Modifications to existing features
* None

#### Bug fixes
* Pull request [#4](https://github.com/epics-motor/motorHytec/pull/4): Only build the example IOC on Linux and VxWorks

## __R1-0-1 (2020-05-12)__
R1-0-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* None

#### Modifications to existing features
* Pull request [#1](https://github.com/epics-motor/motorHytec/pull/2): Compile ``HytecMotorDriver.cpp`` only under Linux/vxWorks

#### Bug fixes
* Commit [b82e4a4](https://github.com/epics-motor/motorHytec/commit/b82e4a4bee64ac7b78f9affd9a15ffb0a2794f1e): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorHytec is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorHytec can be built outside of the motor directory
* motorHytec has a dedicated example IOC that can be built outside of motorHytec

#### Modifications to existing features
* None

#### Bug fixes
* None
