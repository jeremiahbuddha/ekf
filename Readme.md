
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
*Motion* object. 

03/21/2014:

Alright, so today I finished cleaning up how the Action classes compute 
and return the partials. Right now, partial calculation flow works like this:

- Indicate desire to integrate by calling Motion::stepTo()
- Motion::stepTo creates a vector called stateAndStm and passes it into OdeintHelper
- OdeintHelper creates a vector called accel, and loops over all actions and collects
  the accel values
- OdeintHelper creates a vector called partials, and loops over all actions and
  collects the partial values. NOTE that the arrangement of the partials values
  in the partials vector is the same order they appear in the "activeAgents" 
  vector.
- OdeintHelper takes the accel and partials values and puts them returns them
  to the integrator at every time step.

Now I need to:

- Modify my "observer" to also log the STM values.
- Understand what the STM values are (do they map me just back to the previous
  time step, or all the way back to the epoch?)
- Get a printStm() method working on my Motion.  
- Implement the partials of state wrt J2, Cd, etc in my Action classes.
- Start working on my filter (in the knowlege class)
- Write some unit tests?
- Lots of other stuff.
