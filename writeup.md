# FCND-Controls-CPP, Building a controller for Quadrotor. #

This is the writeup for the C++ project. This project helps in stabilizing the desired goal of a quadrotor by calculating errors in desired and actual values of various aspects of the vehicle and adjusting the thrust and moments around each axis accordingly.


For easy navigation throughout this document, here is an outline:

 - [Body Rate Controller](#body-rate-controller)
 - [Roll Pitch Controller](#pitch-roll-controller)
 - [Altitude Controller](#altitude-controller)
 - [Lateral Position Controller](#lateral-position-controller)
 - [Yaw Controller](#yaw-controller)
 - [Generate Motor Commands](#generate-motor-commands)


## Body Rate Controller ##
### Scenario 1 ##
Let us first look at scenario 1 where the mass is adjusted to support the thrust and make the vehicle hover for a bit.
![Scenario 1 Intro](Scenario_1_Intro.mp4) 
Please take a look at file Scenario_1_Intro.mp4.

Body rate controller is implemented in the method `V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)`.
It is a Proportional controller which takes into account the Roll (p), Pitch (q) and Yaw (r) rates of the quadrotor in body frame as opposed to the Global XYZ frame.
It returns the desired moments along xyz axes. Each controller has gains associated with it that can be tuned to achieve desired results.
Angle rate gain: `kpPQR * (pqr_err)`.
## Roll Pitch Controller ##
Roll Pitch controller is implemented in the method `V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)`.
It is a Proportional controller, which takes into account the Global frame acceleration along XY axes, attitude and returns pq(Roll, Pitch) rates in body frame. 
These values will get consumed by Body Rate Controller. There is a Rotational matrix R used for the conversion from global to body frame values.
Angle control gain: `kpBank * bError`.
### Scenario 2 ###
![Scenario 2 Attitude Control](Scenario_2_AttitudeControl.mp4)
Check Scenario_2_AttitudeControl.mp4 for Pitch and roll control.

## Altitude Controller ##
Altitude Controller is implemented in the method `float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)`.

This is a Proportional-Derivative-Integral(PID) controller, which takes in the Z position, velocity and acceleration and returns the collective thrust needed
for the quadrotor to achieve its desired Z position. Proportional term acts on positional Z error.
Derivative term deals with velocity differences. Integral term uses dt and cumulatively adds the positional error.
Roll Pitch Controller takes in the collective thrust outputted from this controller.
Integral term: ` integratedAltitudeError += posZError * dt;
  float integralTerm = KiPosZ * integratedAltitudeError;`.
Position control gains: `float proportionalTerm = kpPosZ * posZError`, KiPosZ.
Velocity control gains: kpVelZ.

## Lateral Position Controller ##
Lateral Position Controller is implemented in method `V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)`.
This is a Proportional-Derivative(PD) controller, which takes in XY position, velocity and feed-forward acceleration. It returns the desired acceleration.
Proportional Term: ` V3F proportionalTerm = posError.operator*(kpPosXY);`.
Derivative Term: ` V3F derivativeTerm = kpVelXY * velDotError;`.
Desired acceleration along XY axes is restricted to range [-maxAccelXY, maxAccelXY], `  accelCmd.constrain(-maxAccelXY, maxAccelXY); `.
The desired acceleration will be used by the Roll Pitch Controller.
Velocity control gains: kpVelXY.

## Yaw Controller ##
Yaw Controller is implemented in the method `float QuadControl::YawControl(float yawCmd, float yaw)`.
It is a Proportional controller, which takes in commanded and actual Yaw in radians and return commanded Yaw rate in radians per second.
First ensure that yaw is within the range 0 to 2 PI.
`yawCmd = fmodf(yawCmd, 2. * F_PI);`. 
Adjust the error accordingly.
Angle control gains: kpYaw.

## Generate Motor Commands ##
`VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)` has the code associated with this feature.
The collective thrust and moments around each axis generated from previous controllers is taken as input.
Output is the calculated thrust for each individual motor.
We need l, the perpendicular distance from x-axis to the motor.
`float l = L / (sqrtf(2.f));`. A little bit of Pythogorean or Baudhayana math will help.


### Scenario 3 ###
![Scenario 3 Position Control](Scenario_3_PositionControl.mp4)
### Scenario 4 ###
![Scenario 4 Non Idealities](Scenario_4_Nonidealities.mp4)
### Scenario 5 ###
![Scenario 5 Trajectory Follow](Scenario_5_TrajectoryFollow.mp4)
The quadrotor finishes its task within the set margin of error.

## Extra 1 ##
The file traj/MakePeriodicTrajectory.py was modified to generate a new trajectory file,
FigureEightFF.txt. The velocities along with x,y,z positional values were returned to generate a trajectory that is crispier.  
![Scenario 5 Trajectory Follow Extra 1](Scenario_5_TrajectoryFollowExtra1.mp4)

## Parameter Tuning ##
QControlParameters.txt has parameters to fine tune gains associated with each controller.
Initially, I started with setting mass to pass Scenario 1.
For scenario 2, P term of kpPQR was amped up. kpBank was also toyed with.
For scenario 3,4 and 5, kpPosXY, kpPosZ, kpVelXY, kpVelZ and kpYaw were fine-tuned to pass.

P term(Roll) was increased from 23 in intervals of 2 and later by 5. Right around 43 it looked stable.
kBank was changed in steps of 5, it looked like the overshoot is gone and time to settle seemed reasonable.
kPosXY and kVelXY were initially tried with the 4 factor suggested for second order systems. Results were not spectacular.
So, amped kPosXY in steps of 5, kVelXY is steps of 0.5 to achieve the desired result.
The Z component was a maverick. I learnt the hard way. I had changed the mass parameter in
QPhysicalParams.txt. Additionally, my initial implementation was based on python solution code.
I changed it to more maintainable, class resources based code, with clear PID terms.
As, the mass was reset in the params, everything fell into place.
QControlParams mass was also tweaked to achieve the desired result.
For KiPosZ, kpPosZ and kpVelZ, I took help of the Hub and mentor suggestions.
For yaw control, r value of kpPQR was set to 6 and modification to kBank was needed to stabilize scenario 2, as roll was failing.
The q component of kPQR was randomly set with trial and error method.
Overall, it has been a nice experience turning into a control freak like Sebastian. 




