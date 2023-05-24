# motorScriptMotor Releases

## __R1-2 (2023-04-13)__
R1-2 is a release based on the master branch.

### Changes since R1-1

#### New features
* Pull request [#1](https://github.com/epics-motor/motorScriptMotor/pull/1): [Keenan Lang](https://github.com/keenanlang) added Attocube AMC scripts and example

#### Modifications to existing features
* Pull request [#3](https://github.com/epics-motor/motorScriptMotor/pull/3): [Oksana Ivashkevych](https://github.com/oksanagit) and [Dave Cyl](https://github.com/davecyl) improved the Attocube AMC support
* Commit [37f476c](https://github.com/epics-motor/motorScriptMotor/commit/37f476c636554e1ff117b15793c129a30d1687a6): SUPPORT has been removed from the RELEASE file; use RELEASE.local to define it instead.

#### Bug fixes
* Commit [2e4080a](https://github.com/epics-motor/motorScriptMotor/commit/2e4080a31848ef924cc700ecd2e452f9178ae56f): Display error details on load failure
* Commit [6c82209](https://github.com/epics-motor/motorScriptMotor/commit/6c822097f6633c7cb0c1483e52a0f28ce08de3d4): Avoid unsupported RTEMS builds

#### Continuous integration
* Added ci-scripts (v3.0.1)
* Configured to use Github Actions for CI

## __R1-1 (2020-05-12)__
R1-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* Commit [194348c](https://github.com/epics-motor/motorScriptMotor/commit/194348c646e1b93cf37c8b58dbeb30dfa25d810b): User displays can now be autoconverted at build time

#### Modifications to existing features
* Commit [79ab30e](https://github.com/epics-motor/motorScriptMotor/commit/79ab30e5babb448e84db1134c15639f9a491ea96): ``CONFIG_SITE`` now includes ``$(SUPPORT)/configure/CONFIG_SITE``, if it exists
* Commit [8d0de84](https://github.com/epics-motor/motorScriptMotor/commit/8d0de8449e0aba7c54c5737fb377eac7d9a1ab09): ``SUPPORT`` is now defined in ``RELEASE``, which is needed for autoconvert
* Commit [61581d0](https://github.com/epics-motor/motorScriptMotor/commit/61581d0cd61be05c34f6ed0c02a179e018fd4bd9): User displays have been autoconverted using the latest converter

#### Bug fixes
* Commit [5517d7d](https://github.com/epics-motor/motorScriptMotor/commit/5517d7deaf04bab1a6057c64a82cd00381ce480e): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorScriptMotor is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorScriptMotor can be built outside of the motor directory
* motorScriptMotor has a dedicated example IOC that can be built outside of motorScriptMotor

#### Modifications to existing features
* None

#### Bug fixes
* None
