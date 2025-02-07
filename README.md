# vortex-gripper

[![Industrial CI](https://github.com/vortexntnu/vortex-gripper/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-gripper/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/vortex-gripper/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/vortex-gripper/main)

## Introduction

This repository contains the ROS 2 package that takes joystick values and converts them to PWM values which is sent to the gripper servos over I2C. The mapping is:
- Left stick vertical controls gripper shoulder
- Left stick horizontal controls gripper arm
- Right stick horizontal controls grip

## Setup

The ROS 2 node can be started with
```bash
ros2 launch gripper_interface gripper_interface.launch.py
```

Joystick is started with
```bash
ros2 run joy joy_node
```
