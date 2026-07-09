# Probing Task

This codebase is an initial version for testing. Once the probing task has been tested and stabilised, it will be transferred to the uol-lab's GitHub repository.

This repository contains the initial implementation of the probing task. The first objective is to implement a set of reusable probing primitives for low-volume material characterisation using a probe.

Each probing primitive should be parameterised by the tool pose, insertion depth, velocity, approach angle, hold time, and force limits. The implementation should be compatible with the robot/ROS 2 pipeline and support synchronised recording of tool motion, force/torque signals, and visual observations.

Code Structure:
