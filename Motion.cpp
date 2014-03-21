
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
     m_stm(),
     m_activeAgents( { "X", "Y", "Z", "dX", "dY", "dZ" } ),
     m_step(),
     m_actions(),
     m_helper( m_actions, m_activeAgents ),
     m_pastStates()
{ 
}

// Constructor with set of intitial conditions 
Motion::                                                                         
Motion( const vector< double > &ic, double step )                                              
   : m_time( 0 ),
     m_state( ic ),                                                                  
     m_stm(),
     m_activeAgents( { "X", "Y", "Z", "dX", "dY", "dZ" } ),                                                           
     m_step( step ),
     m_actions(),
     m_helper( m_actions, m_activeAgents ), 
     m_pastStates()                                                              
{                                                                                
   cout << "CONSTRUCTOR NUMAGENTS " << m_activeAgents.size() << endl;
   initializeStm( m_activeAgents );
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
addAction( Action &a )
{
   Action* ap = &a; 
   m_actions.push_back( ap );
}

// Activate partials tracking for named agents
void
Motion::
activateAgents( const vector< string > agentNames )
{
   for ( string a: agentNames )
   {
      m_activeAgents.push_back( a );
   }

   // Re-initialize the STM to make room for new agents
   initializeStm( m_activeAgents );
}

// Step the integration of Motion object to time t
void                                                                             
Motion::                                                                         
stepTo( double t ) 
{
   // Set up state initial condition
   int stmSize = m_stm.size();
   vector< double > stateAndStm( 6 + stmSize, 0.0 );
   for ( int i = 0; i < 6 ; ++i )
   {
      stateAndStm[i] = m_state[i];
   }
   for ( int i = 0; i < stmSize; ++i )                                                
   {                                                                             
      stateAndStm[6 + i] = m_stm[i];                                          
   }

   using namespace boost::numeric::odeint;

   typedef runge_kutta_dopri5< vector< double > > rkStepper;

   // Integrate from current time to time t                                                        
   integrate_const( make_controlled( 1.E-10, 1.E-9, rkStepper() ), 
                    m_helper, stateAndStm, m_time, t, m_step, 
                    log_state( m_pastStates ) );                

   // Update state, stm, and time
   for ( int i = 0; i < 6 ; ++i )                                                
   {                                                                             
      m_state[i] = stateAndStm[i];                                          
   }
   for ( int i = 0; i < stmSize; ++i )                                           
   {                                                                             
      m_stm[i] = stateAndStm[6 + i]; 
   }   
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
getPartials( double t ) const
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
      cout << setprecision(18) << state[0] << endl;                               
      cout << state[1] << endl;                                                   
      cout << state[2] << endl;                                                   
      cout << state[3] << endl;                                                   
      cout << state[4] << endl;                                                   
      cout << state[5] << endl;  
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
   for ( auto a: m_pastStates )
   {
      printState( a.first );
   }                                                                             
} 

//=============================================================================  
//=============================================================================  
// PRIVATE MEMBERS
void
Motion::
initializeStm( vector< string > &activeAgents )
{
   // Set the "state" part of the STM ( first 6 columns ) to the identity 
   // matrix because dx0/dx0 = I
   int numAgents = activeAgents.size();
   cout << "INITIALIZESTM NUMAGENTS " << numAgents << endl;
   m_stm.resize( 6 * numAgents, 0.0 );
   for ( int i = 0; i < 6 ; ++i )
   {
      m_stm[ 6 * i + i ] = 1; 
   }
}
