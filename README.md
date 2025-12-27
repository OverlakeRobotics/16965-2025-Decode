# Overlake Robotics Library

The Overlake Robotics Library is a movement library meant for FTC teams to control their robot
during both the autonomous and TeleOp period. It works best with an odometry system, especially
GoBilda Pinpoint, but also works with just encoders.

This library is designed to be easy to use, with little to no tuning required and a custom path
planner interface you can use to design your autonomous with zero code.

## Requirements
- Holonomic drivetrain (X-Drive or Mecanum)
- Motor encoders (even if you have odometry)
- Optional but recommended: Odometry system

## Quick Start
1. Fork this repository and clone that fork to your computer.
2. Upload the code to the robot (Guide to do this with ADB: https://ftc-tech-toolbox.vercel.app/docs/Getting%20Started/wd).
3. Run some of the example OpModes and look through them.
4. Optional: Run the tuning programs to get better autonomous movement (Requires an odometry system)
5. Make your own OpModes using the movement library!

## Path Planner Compatability
To make full use of this movement library, it's recommended you use the Overlake Robotics Path Planner,
which you can get here: https://github.com/OverlakeRobotics/OverlakeRoboticsPathPlanner

Using the Overlake Robotics Path Planner, you can upload paths instantly to the robot. The path
planner doesn't just upload movement, but also lets you upload paths with custom actions like
pausing, intaking, or anything else you want your robot to do. This means that after you code
in all the custom actions you want your robot to be able to take, you can create an entire autonomous 
with zero code. To see more on how to make these custom actions, see the PathPlanExample OpMode.

To upload paths in the path planner webapp simply click the upload button (make sure you have the
correct OpMode selected, e.g. PathPlanExample). This will init the OpMode and upload the path. Then
you just have to run it to see your robot move!
