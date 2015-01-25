// -*- coding:utf-8; mode:c++; mode:auto-fill; fill-column:80; -*-

///
/// @file    OdeintHelper.hpp
/// @brief   Interface class between ekf and boost::odeint
/// @author  Jonathon Smith <jonathon.j.smith@gmail.com>
/// @date    January 24, 2015
///

#pragma once
#ifndef EKF_ODEINTHELPER_HEADER_GUARD
#define EKF_ODEINTHELPER_HEADER_GUARD

// C++ Standard Library
#include <string>
#include <vector>

// ekf Library
#include <Action.hpp>

/// @brief Interface class between ekf and boost::odeint.
///
/// This class implements the interface that the odeint library
/// requires. It is used by Motion to manage the state integration.
///
class OdeintHelper{
 public:

  OdeintHelper();
  OdeintHelper( std::vector< std::shared_ptr< Action > >& actions,
                std::vector< std::string >& activeAgents );
 ~OdeintHelper();

  // Allows this class to be called by the odeint solver
  void operator() ( const std::vector< double >& x,
                    std::vector< double >& dxdt,
                    const double t );
  void howManyActions();

 private:
  std::vector< std::shared_ptr< Action > >* m_actions;
  std::vector< std::string >* m_activeAgents;
  /// @todo this needs to go eventually
  const bool m_debug = false;
};

#endif // EKF_ODEINTHELPER_HEADER_GUARD
