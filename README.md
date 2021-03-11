# Kalman Filter Template

This repo is created as an interface class of Filter which can be implemented for both Kalman Filter and Extended Kalman filter

The interface class are in filter.hpp and sensor.hpp which will contains the actions should be taken or potentially be taken as a filter or a sensor.

kalmanFilter.cpp is the class implementing filter interface but it is actually an EKF, with px, py, vx, vy as state

rangeSensor.cpp is the class implementing sensor interface, with range, theta, range_rate as measurement, currently it just assume all raw data process is "magically" done and we already can get the object info under the range sensor pola coordinate frame.

<p>&nbsp;</p>

## How to build
The CmakeList.txt is the instruction, follow normal cmake process will generate executable. Steps as follows
```
//under the source root
mkdir build && cd build
cmake ..
make
```

In the build folder there will be an executable KalmanFilterTemplate
