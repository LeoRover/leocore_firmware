# leocore_firmware

The firmware for the [LeoCore] controller running inside Leo Rover. 

The main functionalities include:
- velocity commands for the robot,
- velocity and PWM commands for individual wheels,
- battery voltage feedback,
- wheel states (position, velocity, PWM duty) feedback,
- odometry feedback (calculated from wheel encoders),
- feedback from the IMU sensor.

It uses [rosserial] client library to expose its functionalities on ROS topics, services and parameters.

[LeoCore]: https://www.leorover.tech/documentation/leo-core