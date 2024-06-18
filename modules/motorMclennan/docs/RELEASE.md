# motorMclennan Releases

## __R1-1-1 (2023-04-11)__
R1-1-1 is a release based on the master branch.

### Changes since R1-1

#### New features
* None

#### Modifications to existing features
* None

#### Bug fixes
* None

#### Continuous integration
* Added ci-scripts (v3.0.1)
* Configured to use Github Actions for CI

## __R1-1 (2020-05-12)__
R1-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* None

#### Modifications to existing features
* Commit [664070d](https://github.com/epics-motor/motorMclennan/commit/664070de78d7840f3f4057b20e5e97d6ac224688): iocsh files are now installed to the top-level directory

#### Bug fixes
* Commit [8a84e30](https://github.com/epics-motor/motorMclennan/commit/8a84e307b9bc2cb608a223bf9a3187934ed475d8): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``
* Pull request [#1](https://github.com/epics-motor/motorMclennan/pull/1): Eliminated compiler warnings

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorMclennan is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorMclennan can be built outside of the motor directory
* motorMclennan has a dedicated example IOC that can be built outside of motorMclennan

#### Modifications to existing features
* None

#### Bug fixes
* None
