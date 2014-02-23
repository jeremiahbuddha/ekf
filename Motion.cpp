

#include <boost/numeric/odeint.hpp>                                              
#include <Motion.hpp>

//=============================================================================
//=============================================================================  
// CONSTRUCTORS / DESCTRUCTOR

// Default Constructor
Motion::
Motion() 
   : m_actions()
{ 
}

//// Constructor with set of InitialCondition's                                  
//Motion::                                                                         
//Motion( const InitialCondition &ic )                                              
//   : m_state(),                                                                  
//     m_actions()                                                                 
//{                                                                                
//   setInitialConditions( ic ); 
//}  

// Default Destructor
Motion::
~Motion() {}

//=============================================================================  
//=============================================================================  
// PUBLIC MEMBERS

// Add an Action 
void
Motion::
addAction( const Action &a )
{
   m_actions.push_back( a );
}

// Function formatted to work with boost/numeric/odeint. Takes in a dimension    
// 6 state vector, and returns the first derivative.
void                                                                             
Motion::                                                                         
operator() (                                                                     
   const vector< double > &x ,                                                   
   vector< double > &dxdt ,                                                      
   double t )                                                                    
{                                                                                
   
   vector< double > accel = { 0, 0, 0 };                                         
   // Accumulate accelerations from the different actions.                                                     
   for ( auto &a: m_actions )                                                    
   {                                                                             
      a.getAcceleration( accel, t, x );                                          
   }                                                                             
                                                                                 
   // State elements                                                             
   dxdt[0] = x[3]; // X_dot                                                      
   dxdt[1] = x[4]; // Y_dot                                                      
   dxdt[2] = x[5]; // Z_dot                                                      
   dxdt[3] = accel[0]; // DX_dot                                                 
   dxdt[4] = accel[1]; // DY_dot                                                 
   dxdt[5] = accel[2]; // DY_dot                                                 
}  

//// Return the current time step.                                                 
//double                                                                           
//Motion::                                                                         
//getTime() const                                                                  
//{                                                                                
//   return m_time;                                                                
//}    

// Return the state of the motion at the current time step.
//vector< double >
//Motion::
//getState() const
//{
//   return m_state;
//}

// Return the partials of the motion wrt a group of agents at the current
// time step.
//vector< double >
//Motion::
//getPartials( const AgentGroup &a ) const
//{
//   vector< double > nothing = {0};
//   // NEED TO IMPLEMENT
//   return nothing;
//}

//=============================================================================  
//=============================================================================  
// PRIVATE MEMBERS

// Set current state to the initial conditions                                   
//void
//Motion::                                                                         
//setInitialConditions( InitialCondition &ic )                               
//{                                                                                
//                                                                                 
//   vector< double > state = ic.getValues("state");                               
//   m_state = state;                                                              
//                                                                                 
//} 
