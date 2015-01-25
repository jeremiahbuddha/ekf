// -*- coding:utf-8; mode:c++; mode:auto-fill; fill-column:80; -*-

///
/// @file    Motion.hpp
/// @brief   Manage the motion of an agent through space.
/// @author  Jonathon Smith <jonathon.j.smith@gmail.com>
/// @date    January 24, 2015
///

// C++ Standard Library
#include <cmath>

// boost Library
#include <boost/numeric/odeint.hpp>

// ekf Library
#include <Motion.hpp>

//=====================================================================
//=====================================================================
// This struct is used to "observe" the integrator and log states.
struct log_state
{
  std::map< double, std::vector< double > >* m_pastStates;

  // Constructor
  log_state(  std::map< double, std::vector< double > >& pastStates )
      : m_pastStates( &pastStates ) { }

  // Takes in state and time from odeint integrate function and logs
  // them in the m_pastStates map.
  void operator()( const std::vector< double >& x, double t )
  {
    m_pastStates->insert( std::pair<double, std::vector< double > >(t,x) );
  }
};

//=====================================================================
//=====================================================================
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
Motion(
    const std::vector< double >& ic,
    double step )
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

//=====================================================================
//=====================================================================
// PUBLIC MEMBERS

// Add an Action
void
Motion::
addAction( std::shared_ptr< Action > a )
{
  m_actions.push_back( a );
  m_helper.howManyActions();
}

// Activate partials tracking for named agents
void
Motion::
activateAgents( const std::vector< std::string > agentNames )
{
  for ( std::string a: agentNames )
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
  std::vector< double > stateAndPartials( 6 + partialsSize, 0.0 );
  for ( int i = 0; i < 6 ; ++i )
  {
    stateAndPartials[i] = m_state[i];
  }
  for ( int i = 0; i < partialsSize; ++i )
  {
    stateAndPartials[6 + i] = m_partials[i];
  }

  using namespace boost::numeric::odeint;

  typedef runge_kutta_dopri5< std::vector< double > > rkStepper;

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
std::vector< double >
Motion::
getState( double t ) const
{
  std::map< double, std::vector< double > >::const_iterator search =
    m_pastStates.find( t );
  if ( search != m_pastStates.end() )
  {
    std::vector< double > stateAndPartials = search->second;
    std::vector< double > state( stateAndPartials.begin(),
                            stateAndPartials.begin() + 6 );
    return state;
  }
  else
  {
    std::cout << "No state at time " << t << "." << std::endl;
    throw;
  }
}

// Return the state partials of the motion wrt a group of agents at
// the current time step ( the partials are dX(t)/dX(t0) )
std::vector< double >
Motion::
getStatePartials( double t ) const
{
  std::map< double, std::vector< double > >::const_iterator search =
    m_pastStates.find( t );
  if ( search != m_pastStates.end() )
  {
    std::vector< double > stateAndPartials = search->second;
    std::vector< double > partials( stateAndPartials.begin() + 6,
                               stateAndPartials.end() );
    return partials;
  }
  else
  {
    std::cout << "No state partials at time " << t << "." << std::endl;
    throw;
  }
}

// Pretty print the state at time t ( must either be current time, or a
// valid logged past time.
void
Motion::
printStateAndPartials( double t ) const
{
  std::map< double, std::vector< double > >::const_iterator search =
    m_pastStates.find( t );
  if ( search != m_pastStates.end() )
  {
    std::vector< double > state = search->second;

    std::cout << "\n### State at time " << t << std::endl
              << "X: " << setprecision(18) << state[0] << std::endl
              << "Y: " << state[1] << std::endl
              << "Z: " << state[2] << std::endl
              << "dX: " << state[3] << std::endl
              << "dY: " << state[4] << std::endl
              << "dZ: " << state[5] << std::endl;

    std::cout << "\n### STM at time " << t << std::endl;
    int stmSize = state.size() - 6;
    int numAgents = sqrt( stmSize );
    for ( int i = 0; i < stmSize; ++i )
    {
      std::cout << "   " << state[6 + i];
      if ( ( i > 0 ) && (i % numAgents == 0 ) )
      {
        std::cout << std::endl;
      }
    }
  }
  else
  {
    std::cout << "No state at time " << t << "." << std::endl;
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

//=====================================================================
//=====================================================================
// PRIVATE MEMBERS
void
Motion::
initializePartials( std::vector< std::string > &activeAgents )
{
  // Reset the partials vector to all zeros
  fill( m_partials.begin(), m_partials.end(), 0.0 );

  // Set the state partials from t0 to t0, i.e. the identity matrix
  int numAgents = activeAgents.size();
  m_partials.resize( numAgents * numAgents, 0.0 );
  for ( int i = 0; i < numAgents ; ++i )
  {
    m_partials[ numAgents * i + i ] = 1;
  }
}
