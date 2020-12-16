# cocp_vehicle

This ros package characterizes the implementation of the control strategy described in the work "Validating a Learning Offline Policy for Path-Tracking on a Car-like Vehicle".

In summary, we provide the implementation of an approximate dynamic programming algorithm for the path-tracking control problem of a car-like vehicle platform.

Version | ROS Distro | Operating System
------------ | ------------- | ------------
1.0 | Kinetic Kame | Ubuntu 16.04 LTS (Xenial)


## Lanekeeping system

The desired path is represented represented by piecewise linear segments. The search for the corresponding point involves sequentially inspecting each path segment to determine if the vehicle lies along that portion of the path. The desired trajectory is known and composed of 2.5 cm sized linear segments arranged in a vector structure, which is given in the configuration files [yaml](https://github.com/alexandremr/cocp_vehicle/tree/main/config)

## Control policy
The convex optimization problem is embedded into the package as a a custom C code [cvxgen](https://github.com/alexandremr/cocp_vehicle/tree/main/include/cvxgen) that compiles into a high-speed solver. We use the CVXGEN tool.
