# Overview
The project is a implementation of a PID controller that helps control the
steering of a vehicle as is goes around a track, given the CTE at various points
in time ( Cross track
error)

## Implementation

- A PID controller is created without any assumptions on the Hyper
  parameters
- Manual tuning of Hyper parameter was tried by setting Ki and Kd 0 and setting Kp  to various values. This method seemed to cumbersome and error prone
- I implemented the twiddle algorithm for tuning the hyper parameters and the
  values of Kp,Ki,Kd [64.590094,0.000000,-38.460250] seems to work. I have
  disabled the twiddle updat with a define as per previous review comments

### Twiddle algorithm
- Twiddle algorithm is written to run alongside the simulator
- One of the hyper parameters in increased ( multiplied by 1.1)
  and the next cte is checked. If the next frame cte is better
  than the previous frame cte. This hyper paramter change is kept
- If the CTE goes up, we revert the change and try to go in the
 oppositte direction
- This is done for P, I and D hyper parameters till the algorithm
  converges

## Reflection
-With the P only controller, the vehicle oscillates more and more and never
stabilizes
-With PD controller, the oscillations decay eventually
-PID controller should have made the car more stable, but PD was enough to drive
 around the track and I stopped here

### Further tuning
- I disabled the I part of the algorithm as introducing it was increasing the oscillations. My understanding of this is, that the I part of the controller is needed to overcome the systemic bias. Perhaps the simulator environment does not have any systemic bias?
- I added a condition in the main.cpp to make sure to slow down the throttle to 0.1 if we are making a turn and the speed is greater than 15

### Issues to be solved
- The car still oscillates a bit on the last turn. But never goes out of track.

### Demo video
- Demo video of the car going around the track. link--> (goo.gl/mYCfsp)


