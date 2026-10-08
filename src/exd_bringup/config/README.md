Each subsystem keeps its own parameters in `src/<package>/config/*.yaml`
so students don't edit each other's files.

Put **whole-drone** overrides here (e.g. a `real_drone.yaml` with hardware
values) and load them in `launch/exd.launch.py` after the package configs.
