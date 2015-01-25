// -*- coding:utf-8; mode:c++; mode:auto-fill; fill-column:80; -*-

///
/// @file    Action.hpp
/// @brief   Base class for defining forces capable of effecting the
///          evolution of Motion objects.
/// @author  Jonathon Smith <jonathon.j.smith@gmail.com>
/// @date    January 24, 2015
///

#ifndef EKF_ACTION_HEADER_GUARD
#define EKF_ACTION_HEADER_GUARD

// C++ Standard Library
#include <vector>

class Action
{
 public:
  // Constructor
  Action(){};

  // Computes the acceleration due to this action and adds it to the
  // passed in vector "acceleration".
  virtual void getAcceleration( std::vector< double > &acceleration,
                                const std::vector< double > &state ) const = 0;

  // Computes the partial derivative of the acceleration terms and owned
  // parameters
  virtual void getPartials( std::vector < double > &partials,
                            const std::vector< double > &state,
                            const std::vector< std::string >  &activeAgents ) = 0;
  // Destructor
  virtual ~Action(){};

  protected:
    /// @todo this needs to go eventually
    const bool m_debug = false;

  private:
};

#endif // EKF_ACTION_HEADER_GUARD
