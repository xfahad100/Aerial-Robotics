# Aerial-Robotics
This is a repo for Coursera's Aerial Robotics

This assignment was focused on obtaining linearized equation of motions for quadrotor in 2D plan (Y-Z). A simple PD controller has been used in each plant (phi, u1, u2).

The resulting equations and PD controller tuned parameters are shown below:
![Tuned parameters and control law equations](2D-planar-control-Quadrotor/docs/eqmotion.png)

StraightLine trajectory output:
![tune PD such that drone stables at y=4, z=0](2D-planar-control-Quadrotor/docs/result-line.png)

Sinewave trajectory output:
![notice that drone covers 1 cycle around y=2](2D-planar-control-Quadrotor/docs/result-sine1.png)

My position error based on PD params:
![Tuned parameters and control law equations](2D-planar-control-Quadrotor/docs/pos-error.png)
