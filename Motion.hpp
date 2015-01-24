// -*- coding:utf-8; mode:c++; mode:auto-fill; fill-column:80; -*-

///
/// @file    Motion.hpp
/// @brief   Manage the motion of an agent through space.
/// @author  Jonathon Smith <jonathon.j.smith@gmail.com>
/// @date    January 24, 2015
///

#pragma once
#ifndef EKF_MOTION_HEADER_GUARD
#define EKF_MOTION_HEADER_GUARD

// C++ Standard Library
#include <vector>
#include <map>

// Eigen Library
#include <Eigen/Dense>

// ekf Library
#include <Action.hpp>
#include <AgentGroup.hpp>
#include <OdeintHelper.hpp>

using namespace std;

/// @brief Manage the motion of an agent through space.
///
/// Given a set of Actions, Motion will step the agent forward in time
/// and allow querying the state of the agent at any time it has
/// previously visited.
///
class Motion {

 public:
  Motion();
  Motion( const vector< double > &ic, double step );
 ~Motion();

  // Step to time t
  void stepTo( double t );

  // Add effect of action to motion
  void addAction( Action& a );
  // Activate agents for partials computations
  void activateAgents( const vector< string > agentNames );

  // Get current time step
  double getTime() const;
  // Get value of state at step t ( defaults to current time )
  vector< double > getState( double t ) const;
  // Get the partials of state at step t
  vector< double > getStatePartials( double t ) const;

  // Print the current state to cout
  void printStateAndPartials( double t ) const;
  void printAllStates() const;

 private:

  double m_time;
  vector< double > m_state;
  vector< double > m_partials;
  vector< string > m_activeAgents;
  double m_step;
  vector< Action* > m_actions;
  OdeintHelper m_helper;
  map< double, vector< double > > m_pastStates;

  void initializePartials( vector< string > &activeAgents );
};

#endif // EKF_MOTION_HEADER_GUARD
