
#include <boost/numeric/odeint.hpp>                                              
#include <Motion.hpp>

//=============================================================================
//=============================================================================  
// CONSTRUCTORS / DESCTRUCTOR

// Default Constructor
Motion::
Motion() 
   : m_time(),
     m_state(), 
     m_actions(),
     m_helper()
{ 
}

// Constructor with set of intitial conditions 
Motion::                                                                         
Motion( const vector< double > &ic )                                              
   : m_time( 0 ),
     m_state( ic ),                                                                  
     m_actions(),
     m_helper()                                                                 
{                                                                                
}  

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
   const Action* ap = &a; 
   m_actions.push_back( ap );
   m_helper.addAction( a );
}

// Step the integration of Motion object to time t
void                                                                             
Motion::                                                                         
step( double t ) 
{
   using namespace boost::numeric::odeint;

   // state_type = double
   typedef runge_kutta_dopri5< vector< double > > stepper_type;

   // Integrate to time t                                                        
   integrate_adaptive( make_controlled( 1.E-10, 1.E-9, stepper_type() ), m_helper, m_state, 
                     m_time, t, 1.0 );                                                                                
   m_time = t;

}  

// Return the current time step.                                                 
double                                                                           
Motion::                                                                         
getTime() const                                                                  
{                                                                                
   return m_time;                                                                
}    

// Return the state of the motion at the current time step.
vector< double >
Motion::
getState() const
{
   return m_state;
}

// Return the partials of the motion wrt a group of agents at the current
// time step.
vector< double >
Motion::
getPartials( const AgentGroup &a ) const
{
   vector< double > nothing = {0};
   // NEED TO IMPLEMENT
   return nothing;
}

void
Motion::
printState() const
{
   cout << "\n### State at time " << m_time << "\n";                                          
   cout << setprecision(18) << m_state[0] << "\n";                                 
   cout << m_state[1] << "\n";                                                     
   cout << m_state[2] << "\n";                                                     
   cout << m_state[3] << "\n";                                                     
   cout << m_state[4] << "\n";                                                     
   cout << m_state[5] << "\n";    
}

//=============================================================================  
//=============================================================================  
// PRIVATE MEMBERS

