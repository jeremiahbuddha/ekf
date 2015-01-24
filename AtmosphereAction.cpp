// -*- coding:utf-8; mode:c++; mode:auto-fill; fill-column:80; -*-

///
/// @file    AtmosphereAction.cpp
/// @brief   Computes state accelerations and partials due to the
///          interaction of an agent with a planetary atmosphere.
/// @author  Jonathon Smith <jonathon.j.smith@gmail.com>
/// @date    January 24, 2015
///

// C++ Standard Library
#include <cmath>
#include <iostream>

// ekf Library
#include <AtmosphereAction.hpp>

using namespace std;
//=====================================================================
//=====================================================================
// CONSTRUCTORS / DESCTRUCTOR

// Default Constructor
AtmosphereAction::
AtmosphereAction()
    : m_name(),
      m_refHeight(),
      m_refDensity(),
      m_stepHeight(),
      m_rotation(),
      m_bodyDragTerm(),
      m_evaledPartials()
{
}

// Constructor for standard planetary atmosphere
AtmosphereAction::
AtmosphereAction(
    const string name,
    double refHeight,
    double refDensity,
    double stepHeight,
    double rotation,
    double bodyDragTerm )
    : m_name( name ),
      m_refHeight( refHeight ),
      m_refDensity( refDensity ),
      m_stepHeight( stepHeight ),
      m_rotation( rotation ),
      m_bodyDragTerm( bodyDragTerm ),
      m_evaledPartials()
{
}

// Default Destructor
AtmosphereAction::
~AtmosphereAction()
{
}

//=====================================================================
//=====================================================================
// PUBLIC MEMBERS

// Computes the acceleration due to drag from a planetary body
// atmosphere.
void
AtmosphereAction::
getAcceleration(
    vector< double > &acceleration,
    const vector< double > &state ) const
{
  double dragPrefix =  - m_bodyDragTerm * adjustedDensity( state )
                       * adjustedVelocity( state );

  acceleration[0] += dragPrefix * ( state[3] + state[1] * m_rotation );
  acceleration[1] += dragPrefix * ( state[4] - state[0] * m_rotation );
  acceleration[2] += dragPrefix * ( state[5] );
}

// Computes the partial derivative of the acceleration terms and owned
// parameters
void
AtmosphereAction::
getPartials(
    vector< double > &partials,
    const vector< double > &state,
    const vector< string >  &activeAgents )
{
  // Evaluate the class partial for this state
  evalPartials( state );

  // Loop over active agents and get partial values
  int numAgents = activeAgents.size();
  for ( int i = 0; i < numAgents; ++i )
  {
    // Request the partial from the i loop with respect to all the
    // active agents in j loop
    for ( int j = 0; j < numAgents; ++j )
    {
      if (m_debug)
      {
        cout << "\nAtmosphereAction::getPartials()" << endl
             << "Requested Partials: " << activeAgents[i] <<  " wrt "
             << activeAgents[j] << endl
             << "Value of partials: "
             << getAgentPartial( activeAgents[i], activeAgents[j] );
      }
      partials[ i * numAgents + j ] += getAgentPartial( activeAgents[i],
                                                        activeAgents[j] );
    }
  }
}

//=====================================================================
//=====================================================================
// PRIVATE MEMBERS

// Get the atmospheric density at current state
double
AtmosphereAction::
adjustedDensity( const vector< double > state ) const
{
  double dist = sqrt( pow( state[0], 2 ) + pow( state[1], 2 ) +
                pow( state[2], 2 ) );

  return m_refDensity * exp( - ( dist - m_refHeight ) / m_stepHeight );
}

// Get the atmospheric relative velocity at current state
double
AtmosphereAction::
adjustedVelocity( const vector< double > state ) const
{
  return sqrt( pow( state[3] + state[1] * m_rotation, 2 ) +
               pow( state[4] - state[0] * m_rotation, 2 ) +
               pow( state[5], 2 ) );
}

double
AtmosphereAction::
getAgentPartial(
    const string &top,
    const string &bottom )
{
  // Form param search string
  string partialRequest = top + " wrt " + bottom;

  if( m_evaledPartials.find( partialRequest ) == m_evaledPartials.end() )
  {
    // If requested partial is not supported by this action, return 0
    return 0.0;
  }
  return m_evaledPartials[ partialRequest ];
}

void
AtmosphereAction::
evalPartials( const vector< double > &state )
{
  // Condense variable names to make following equations more legible
  double r = sqrt( pow( state[0], 2 ) + pow( state[1], 2 ) +
             pow( state[2], 2 ) );
  double X = state[0];
  double Y = state[1];
  double Z = state[2];
  double dX = state[3];
  double dY = state[4];
  double dZ = state[5];
  double step = m_stepHeight;
  double rot =  m_rotation;
  double rho = adjustedDensity( state );
  double vel = adjustedVelocity( state );
  double Cd = m_bodyDragTerm;

  if (m_debug)
  {
    cout << "In AtmosphereAction::evalPartials " << endl
         << "Val of vel: " << vel << endl
         << "Val of rho: " << rho << endl
         << "Val of cd: " << Cd << endl;
  }

  m_evaledPartials[ "X wrt dX" ] = 1;
  m_evaledPartials[ "Y wrt dY" ] = 1;
  m_evaledPartials[ "Z wrt dZ" ] = 1;

  // Partials of acceleration X component wrt state.
  m_evaledPartials[ "dX wrt X" ] = (
    Cd * rho * vel * X * ( dX + rot * Y ) / ( r * step ) +
   -Cd * rho * ( -rot * dY + pow( rot, 2 ) * X ) * ( dX + rot * Y ) / vel );
  m_evaledPartials[ "dX wrt Y" ] = (
    Cd * rho * vel * Y * ( dX + rot * Y ) / ( r * step ) +
   -Cd * rho * ( rot * dX + pow( rot, 2 ) * Y ) * ( dX + rot * Y ) / vel +
   -Cd * rho * vel * rot );
  m_evaledPartials[ "dX wrt Z" ] =
    Cd * rho * vel * Z * ( dX + rot * Y ) / ( r * step );
  m_evaledPartials[ "dX wrt dX" ] =
   -Cd * rho * pow( dX + rot * Y, 2 ) / vel - Cd * rho * vel ;
  m_evaledPartials[ "dX wrt dY" ] =
   -Cd * rho * ( dY - rot * X ) * ( dX + rot * Y ) / vel;
  m_evaledPartials[ "dX wrt dZ" ] =
   -Cd * rho * dZ * ( dX + rot * Y ) / vel;

  // Partials of acceleration Y component wrt state.
  m_evaledPartials[ "dY wrt X" ] = (
    Cd * rho * vel * X * ( dY - rot * X ) / ( r * step ) +
   -Cd * rho * ( pow( rot, 2 ) * X - rot * dY ) * ( dY - rot * X ) / vel +
    Cd * rho * vel * rot );
  m_evaledPartials[ "dY wrt Y" ] = (
    Cd * rho * vel * Y * ( dY - rot * X ) / ( r * step) +
   -Cd * rho * ( rot * dX + pow( rot, 2 ) * Y ) * ( dY - rot * X ) / vel );
  m_evaledPartials[ "dY wrt Z" ] =
    Cd * rho * vel * Z * ( dY - rot * X ) / ( r * step );
  m_evaledPartials[ "dY wrt dX" ] =
   -Cd * rho * ( dY - rot * X ) * ( dX + rot * Y ) / vel;
  m_evaledPartials[ "dY wrt dY" ] =
   -Cd * rho * pow( dY - rot * X, 2 ) / vel - Cd * rho * vel;
  m_evaledPartials[ "dY wrt dZ" ] =
   -Cd * rho * dZ * ( dY - rot * X ) / vel;

  // Partials of acceleration Z component wrt state.
  m_evaledPartials[ "dZ wrt X" ] = (
    Cd * rho * vel * dZ * X / (r * step) +
   -Cd * rho * dZ * ( pow( rot, 2 ) * X - rot * dY ) / vel );
  m_evaledPartials[ "dZ wrt Y" ] = (
    Cd * rho * vel * dZ * Y / ( r * step ) +
   -Cd * rho * dZ * ( rot * dX + pow( rot, 2 ) * Y) / vel );
  m_evaledPartials[ "dZ wrt Z" ] =
    Cd * rho * vel * Z * dZ / ( r * step );
  m_evaledPartials[ "dZ wrt dX" ] =
   -Cd * rho * dZ * ( dX + rot * Y ) / vel;
  m_evaledPartials[ "dZ wrt dY" ] =
   -Cd * rho * dZ * ( dY - rot * X ) / vel;
  m_evaledPartials[ "dZ wrt dZ" ] = (
   -Cd * rho * pow( dZ, 2 ) / vel ) + ( -Cd * rho * vel );

/// @todo implement remaining partials:
///   - Cartesian state X-component
///   - Cartesian state Y-component
///   - Cartesian state Z-component
///   - Cartesian state dX-component
///   - Cartesian state dY-component
///   - Cartesian state dZ-component
///   - Exponential atmosphere referece height
///   - Exponential atmosphere reference density
///   - Exponential atmosphere step height
///   - Planetary rotation
///   - Agent body drag term
}
