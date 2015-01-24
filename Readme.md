
# EKF

## Package Description

EKF implements an extended kalman filter for estimating the motion of an object
through space using tracking data.

### Class *Motion*

The *Motion* class manages the motion of some object through space. It is 
responsible for managing the effect of added *Action* objects, and returning
the acceleration driving motion at any time. It is also responsible for 
computing the partials of the Motion with respect to any *Agent* at any
requested time.

### Class *Action*

The *Action* class defines a force capable of effecting the evolution of a
*Motion* object. It is the responsibility of the Action classes to define
the state partial derivatives! - as well as the partial derivatives of
any quantities they define with respect to all dependent parameters. 

NOTE: Google C++ Style says to comment on class definintions (not 
declarations), but I dont think that makes sense here. I will provide
a high-level overview of the class as a preamble comment, and then
add comment documentation to class declarations (hpp).

03/21/2014:

Alright, so today I finished cleaning up how the Action classes compute 
and return the partials. Right now, partial calculation flow works like this:

- Indicate desire to integrate by calling Motion::stepTo()
- Motion::stepTo creates a vector called stateAndStm that has the current state
  and STM value, and passes it into OdeintHelper
- OdeintHelper works iteratively with the integrator. Give the current state,
  it loops over all actions and collects the accel values. Given the current 
  STM, it also loops over all the actions and gets the parameter partials. It 
  compiles all the partials into an A matrix, and performs A * currentSTM to get 
  the derivative of the STM.
- OdeintHelper takes the accel and derivative of STM and puts them returns them
  to the integrator at every time step.

Now I need to:

- X Modify my "observer" to also log the STM values.
- X Understand what the STM values are (do they map me just back to the previous
    time step, or all the way back to the epoch?) They map all the way back to 
    the epoch.
- X Get a printStatePartials() method working on my Motion.  
- X Multiply the "partials" matrix in OdeintHelper ( which is really the A 
    matrix ) by the STM part of the state vector to get the derivative of
    the STM.
- X Verify partials at t=10 against python version.
- Implement the partials of state wrt J2, Cd, etc in my Action classes.
- Start working on my filter (in the knowlege class)
- Write some unit tests?
- Lots of other stuff.
