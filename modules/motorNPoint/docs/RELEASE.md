# motorNPoint Releases

## __R1-1 (2023-04-12)__
R1-1 is a release based on the master branch.

### Changes since R1-0-1

#### New features
* Pull request [#1](https://github.com/epics-motor/motorNPoint/pull/1): [Diego Omitto](https://github.com/domitto) added support for the nPoint LC400 controller

#### Modifications to existing features
* None

#### Bug fixes
* Pull request [#3](https://github.com/epics-motor/motorNPoint/pull/3): [Diego Omitto](https://github.com/domitto) fixed an off-by-one error on WAV_DELAY when calculating velocity

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
* Commit [f935b02](https://github.com/epics-motor/motorNPoint/commit/f935b02bd55ebc255daa1df9407dbd6cd0b6cd40): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorNPoint is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorNPoint can be built outside of the motor directory
* motorNPoint has a dedicated example IOC that can be built outside of motorNPoint

#### Modifications to existing features
* None

#### Bug fixes
* None
