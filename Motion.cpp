
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
     m_partials(),
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
     m_partials(),
     m_activeAgents( { "X", "Y", "Z", "dX", "dY", "dZ" } ),                                                           
     m_step( step ),
     m_actions(),
     m_helper( m_actions, m_activeAgents ), 
     m_pastStates()                                                              
{                                                                                
   initializePartials( m_activeAgents );
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
   m_helper.howManyActions();
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

   // Re-initialize the partials to make room for new agents
   initializePartials( m_activeAgents );
}

// Step the integration of Motion object to time t
void                                                                             
Motion::                                                                         
stepTo( double t ) 
{
   // Set up state initial condition
   int partialsSize = m_partials.size();
   vector< double > stateAndPartials( 6 + partialsSize, 0.0 );
   for ( int i = 0; i < 6 ; ++i )
   {
      stateAndPartials[i] = m_state[i];
   }
   for ( int i = 0; i < partialsSize; ++i )                                                
   {                                                                             
      stateAndPartials[6 + i] = m_partials[i];                                          
   }

   using namespace boost::numeric::odeint;

   typedef runge_kutta_dopri5< vector< double > > rkStepper;

   // Integrate from current time to time t                                                        
   integrate_const( make_controlled( 1.E-10, 1.E-9, rkStepper() ), 
                    m_helper, stateAndPartials, m_time, t, m_step, 
                    log_state( m_pastStates ) );                

   // Update state, partials, and time
   for ( int i = 0; i < 6 ; ++i )                                                
   {                                                                             
      m_state[i] = stateAndPartials[i];                                          
   }
   for ( int i = 0; i < partialsSize; ++i )                                           
   {                                                                             
      m_partials[i] = stateAndPartials[6 + i]; 
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
      vector< double > stateAndPartials = search->second;
      vector< double > state( stateAndPartials.begin(), 
                              stateAndPartials.begin() + 6 );
      return state;
   }
   else 
   {
      cout << "No state at time " << t << "." << endl;
      throw;
   }
}

// Return the state partials of the motion wrt a group of agents at 
// the current time step ( the partials are dX(t)/dX(t0) )
vector< double >
Motion::
getStatePartials( double t ) const
{
   map< double, vector< double > >::const_iterator search = m_pastStates.find( t );
   if ( search != m_pastStates.end() )                                           
   {                                                                             
      vector< double > stateAndPartials = search->second;                        
      vector< double > partials( stateAndPartials.begin() + 6,                          
                                 stateAndPartials.end() );                    
      return partials;                                                              
   }                                                                             
   else                                                                          
   {                                                                             
      cout << "No state partials at time " << t << "." << endl;                           
      throw;                                                                     
   }       
}

// Pretty print the state at time t ( must either be current time, or a valid
// logged past time.
void
Motion::
printStateAndPartials( double t ) const
{
   map< double, vector< double > >::const_iterator search = 
      m_pastStates.find( t );
   if ( search != m_pastStates.end() )                                           
   {                                                                             
      vector< double > state = search->second;                                                     
      cout << "\n### State at time " << t << endl;                                  
      cout << setprecision(18) << state[0] << endl;                               
      for ( int i = 1; i < state.size(); ++i )
      { 
         cout << state[i] << endl;  
      }
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
      printStateAndPartials( a.first );
   }                                                                             
} 

//=============================================================================  
//=============================================================================  
// PRIVATE MEMBERS
void
Motion::
initializePartials( vector< string > &activeAgents )
{
   // Reset the partials vector to all zeros
   fill( m_partials.begin(), m_partials.end(), 0.0 );
   // Set the state partials from t0 to t0, i.e. the identity matrix
   int numAgents = activeAgents.size();
   m_partials.resize( 6 * numAgents, 0.0 );
   for ( int i = 0; i < 6 ; ++i )
   {
      m_partials[ numAgents * i + i ] = 1; 
   }
}
