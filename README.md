# Modeling-and-Control-of-MIP-Robot

Course Project for MAE 280A Linear Systems Theory, taken during Fall 2019.

## Description

This project aims to model and control the MiP (Mobile Inverted Pendulum) Robot, developed by WowWee Robotics, in collaboration\
with UCSD Coordinated Robotics Lab headed by Professor Tom Bewley.\
\
The task is to design a discrete linear state-estimate feedback controller for MiP that performs decently. \
\
The system has 4 states θ , ϕ , θdot , ϕdot  . Our task is to design a controller for this system such that\
we get a linear quadratic gaussian controller consisting of a linear feedback controller coupled with a state observer.\
The problem requires us to experiment with the eigen values of the feedback and the observer system, \
and to arrive at the optimal controller parameters. We are trying to control the reduced-order system,\
as θ plays no role in the the system's model.


### Code Organisation

```
MIPParameters.m            -- Initialises a list of parameters in the workspace.
nLMIp2.slx                 -- Simulink model containing th non-linear MiP model.
VenkateshPrasadVReport.pdf -- Detailed report of the experiments conducted to arrive at the optimum eigen values for the controller.
```




