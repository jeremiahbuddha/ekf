
#include <boost/numeric/odeint.hpp>                                              
#include <Motion.hpp>

//=============================================================================  
//=============================================================================  
// This struct is used to "observe" the integrator, and log states
struct log_state
{
   map< double, vector< double > > &m_pastStates;

   // Constructor
   log_state(  map< double, vector< double > > &pastStates )
      : m_pastStates( pastStates ) { }   
                                                                         
   // Takes in state and time from odeint integrate function and logs them
   // in the m_pastStates map. 
   void operator()( const vector< double > &x, double t )                                                                    
   {                                                                                
      m_pastStates[t] = x;                                                          
   }
};

//=============================================================================
//=============================================================================  
// CONSTRUCTORS / DESCTRUCTOR

// Default Constructor
Motion::
Motion() 
   : m_time(),
     m_state(), 
     m_step(),
     m_actions(),
     m_helper()
{ 
}

// Constructor with set of intitial conditions 
Motion::                                                                         
Motion( const vector< double > &ic, double step )                                              
   : m_time( 0 ),
     m_state( ic ),                                                                  
     m_step( step ),
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
stepTo( double t ) 
{
   using namespace boost::numeric::odeint;

   typedef runge_kutta_dopri5< vector< double > > rkStepper;

   // Integrate from current time to time t                                                        
   integrate_const( make_controlled( 1.E-10, 1.E-9, rkStepper() ), 
                    m_helper, m_state, m_time, t, m_step, 
                    log_state( m_pastStates ) );                
   m_time = t;
}  

// Return the current time step.                                                 
double                                                                           
Motion::                                                                         
getTime() const                                                                  
{                                                                                
   return m_time;                                                                
}    

// Return the state of the motion at  time step.
vector< double >
Motion::
getState( double t ) const
{
   map< double, vector< double > >::const_iterator search = m_pastStates.find( t );
   if ( search != m_pastStates.end() ) 
   {
      return search->second;
   }
   else 
   {
      cout << "No state at time " << t << "." << endl;
      throw;
   }
}

// Return the partials of the motion wrt a group of agents at the current
// time step.
vector< double >
Motion::
getPartials( double t, const AgentGroup &a ) const
{
   vector< double > nothing = {0};
   // NEED TO IMPLEMENT
   return nothing;
}

// Pretty print the state at time t ( must either be current time, or a valid
// logged past time.
void
Motion::
printState( double t ) const
{
   map< double, vector< double > >::const_iterator search = 
      m_pastStates.find( t );
   if ( search != m_pastStates.end() )                                           
   {                                                                             
      vector< double > state = search->second;                                                     
      cout << "\n### State at time " << t << endl;                                  
      cout << setprecision(18) << m_state[0] << endl;                               
      cout << m_state[1] << endl;                                                   
      cout << m_state[2] << endl;                                                   
      cout << m_state[3] << endl;                                                   
      cout << m_state[4] << endl;                                                   
      cout << m_state[5] << endl;  
   }                                                                             
   else                                                                          
   {                                                                             
      cout << "No state at time " << t << "." << endl;                           
      throw;                                                                     
   }     
}

// Pretty print all states in the log
void                                                                             
Motion::                                                                         
printAllStates() const                                                     
{                                                                                
   cout << "IN HERE" << endl;
   cout << "pastStates is empty: " << m_pastStates.empty() << endl;
   //map< double, vector< double > >::const_iterator all;
   for ( auto a: m_pastStates )
   {
      printState( a.first );
   }                                                                             
} 

//=============================================================================  
//=============================================================================  
// PRIVATE MEMBERS
