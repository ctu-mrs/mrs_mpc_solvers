# MRS MPC solvers

Containts MPC solvers for [mrs_uav_trackers](https://github.com/ctu-mrs/mrs_uav_trackers) and [mrs_uav_controllers](https://github.com/ctu-mrs/mrs_uav_controllers).

## building it by hand

```bash
mkdir build
cd build
cmake ..
make
```

## Extracting the .so files

Copy the `.so` files from `build/devel/lib` to appropriate folders within **mrs_uav_trackers** and **mrs_uav_controllers**.
