
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

