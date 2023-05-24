# motorOmsAsyn Releases

## __R1-0-3 (2023-04-12)__
R1-0-3 is a release based on the master branch.

### Changes since R1-0-2

#### New features
* None

#### Modifications to existing features
* None

#### Bug fixes
* Pull request [#6](https://github.com/epics-motor/motorOmsAsyn/pull/6): Type fixes from [Dirk Zimoch](https://github.com/dirk-zimoch)

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
* Pull request [#5](https://github.com/epics-motor/motorOmsAsyn/pull/5): Exclude MAXv support when building example IOC on Windows

## __R1-0-1 (2020-05-12)__
R1-0-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* None

#### Modifications to existing features
* Pull request [#3](https://github.com/epics-motor/motorOmsAsyn/pull/3): Compile ``omsMAXv.cpp`` only under Linux/vxWorks

#### Bug fixes
* Commit [4fdd003](https://github.com/epics-motor/motorOmsAsyn/commit/4fdd003bcede7728327026525ad633beab6cfcaf): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorOmsAsyn is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorOmsAsyn can be built outside of the motor directory
* motorOmsAsyn has a dedicated example IOC that can be built outside of motorOmsAsyn

#### Modifications to existing features
* None

#### Bug fixes
* None
