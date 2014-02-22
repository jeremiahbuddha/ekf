
#include <Motion.hpp>

// Default Constructor
Motion::
Motion() 
{ 
}

// Default Destructor
Motion::
~Motion() {}

// Create Motion with set of InitialCondition's
Motion::
Motion( const InitialCondition ic )
{
   setInitialConditions( ic )
}


// Set current state to the initial conditions
Motion::
setInitialConditions( const InitialCondition ic )
{

   vector< double > state = ic.getValues("state");
   m_state = state;

};

vector< double >
Motion::
getAcceleration( const Time t )
{
   vector< double > state_accel;

   // The idea here is that state_accel accumulates acceleration from the
   // the different forces. Not sure if this will work?
   for ( auto &f: m_forces )
   {
      f.get_acceleration( t, m_state, state_accel );
   }
   
   return state_accel;
};
