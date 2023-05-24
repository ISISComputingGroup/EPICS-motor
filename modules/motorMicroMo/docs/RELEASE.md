# motorMicroMo Releases

## __R1-0-2 (2023-04-12)__
R1-0-2 is a release based on the master branch.

### Changes since R1-0-1

#### New features
* None

#### Modifications to existing features
* None

#### Bug fixes
* None

#### Continuous integration
* Added ci-scripts (v3.0.1)
* Configured to use Github Actions for CI

## __R1-0-1 (2020-05-11)__
R1-0-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* None

#### Modifications to existing features
* None

#### Bug fixes
* Commit [c1b16db](https://github.com/epics-motor/motorMicroMo/commit/c1b16db9a8af59b85ce1253f7482a880b1b92103): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``
* Commit [04331b5](https://github.com/epics-motor/motorMicroMo/commit/04331b5cfa223a530591a6f2273b414531d8d151): Eliminated compiler warnings

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorMicroMo is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorMicroMo can be built outside of the motor directory
* motorMicroMo has a dedicated example IOC that can be built outside of motorMicroMo

#### Modifications to existing features
* None

#### Bug fixes
* None
