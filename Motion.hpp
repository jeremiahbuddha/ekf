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
#include <string>

// Eigen Library
#include <Eigen/Dense>

// ekf Library
#include <Action.hpp>
#include <AgentGroup.hpp>
#include <OdeintHelper.hpp>

/// @brief Manage the motion of an agent through space.
///
/// Given a set of Actions, Motion will step the agent forward in time
/// and allow querying the state of the agent at any time it has
/// previously visited.
///
class Motion {

 public:
  Motion();
  Motion( const std::vector< double > &ic, double step );
 ~Motion();

  // Step to time t
  void stepTo( double t );

  // Add effect of action to motion
  void addAction( std::shared_ptr<Action> a );
  // Activate agents for partials computations
  void activateAgents( const std::vector< std::string > agentNames );

  // Get current time step
  double getTime() const;
  // Get value of state at step t ( defaults to current time )
  std::vector< double > getState( double t ) const;
  // Get the partials of state at step t
  std::vector< double > getStatePartials( double t ) const;

  // Print the current state to cout
  void printStateAndPartials( double t ) const;
  void printAllStates() const;

 private:

  double m_time;
  std::vector< double > m_state;
  std::vector< double > m_partials;
  std::vector< std::string > m_activeAgents;
  double m_step;
  std::vector< std::shared_ptr< Action > > m_actions;
  OdeintHelper m_helper;
  map< double, std::vector< double > > m_pastStates;

  void initializePartials( std::vector< std::string >& activeAgents );
};

#endif // EKF_MOTION_HEADER_GUARD
