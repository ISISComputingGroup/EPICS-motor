# motorOms Releases

## __R1-2 (2023-04-12)__
R1-2 is a release based on the master branch.

### Changes since R1-1

#### New features
* Pull request [#8](https://github.com/epics-motor/motorOms/pull/8): [Dirk Zimoch](https://github.com/dirk-zimoch) enabled non-VxWorks IOCs

#### Modifications to existing features
* Commit [db0a1c3](https://github.com/epics-motor/motorOms/commit/db0a1c31cb67c579b8110cfc13c2d83bf63371da): SUPPORT has been removed from the RELEASE file; use RELEASE.local to define it instead.

#### Bug fixes
* Pull request [#7](https://github.com/epics-motor/motorOms/pull/7): [Heinz Junkes](https://github.com/hjunkes) fixed an epicsMutexTryLock bug, best explained by [Andrew Johnson](https://github.com/anjohnson) in [this comment](https://github.com/epics-modules/motor/issues/172#issuecomment-758139901)
* Pull request [#9](https://github.com/epics-motor/motorOms/pull/9): [Dirk Zimoch](https://github.com/dirk-zimoch) fixed compiler warnings about unused variables

#### Continuous integration
* Added ci-scripts (v3.0.1)
* Configured to use Github Actions for CI

## __R1-1 (2020-05-12)__
R1-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* Commit [295cadb](https://github.com/epics-motor/motorOms/commit/295cadb68ad20346fb3086d9d66b517ed415fe0b): User displays can now be autoconverted at build time

#### Modifications to existing features
* Commit [671b0ac](https://github.com/epics-motor/motorOms/commit/671b0acf3632c94dd85d4cdad3d48adbdbb746a5): ``CONFIG_SITE`` now includes ``$(SUPPORT)/configure/CONFIG_SITE``, if it exists
* Commit [bef5c87](https://github.com/epics-motor/motorOms/commit/bef5c872c3f524e87d8e8c6f75b5ee1e11ffc5f7): ``SUPPORT`` is now defined in ``RELEASE``, which is needed for autoconvert
* Commit [c583f11](https://github.com/epics-motor/motorOms/commit/c583f112815114449d4b1993efb9056dc09d23e8): User displays have been autoconverted using the latest converter

#### Bug fixes
* Pull request [#3](https://github.com/epics-motor/motorOms/pull/3): Fix cast from ``void*`` to ``epicsUInt32`` loses precision
* Commit [53030c9](https://github.com/epics-motor/motorOms/commit/53030c9fce285196d1d7aac75b61682f597a3a09): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``
* Pull request [#2](https://github.com/epics-motor/motorOms/pull/2): Eliminated compiler warnings

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorOms is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorOms can be built outside of the motor directory
* motorOms has a dedicated example IOC that can be built outside of motorOms

#### Modifications to existing features
* None

#### Bug fixes
* None

