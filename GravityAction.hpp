// -*- coding:utf-8; mode:c++; mode:auto-fill; fill-column:80; -*-

///
/// @file    GravityAction.hpp
/// @brief   Computes state accelerations and partials due to the
///          interaction of an agent with a gravitational body.
/// @author  Jonathon Smith <jonathon.j.smith@gmail.com>
/// @date    January 24, 2015
///

#pragma once
#ifndef EKF_GRAVITYACTION_HEADER_GUARD
#define EKF_GRAVITYACTION_HEADER_GUARD

// C++ Standard Library
#include <string>
#include <vector>
#include <map>

// ekf Library
#include <Action.hpp>

/// @brief Compute state accelerations and partial derivates due to
/// the interaction of an agent and gravitational body.
///
/// This is a resource class for Motion, used to compute the
/// accelerations and partial derivatives wrt a gravitational body
/// at a given Epoch / State.
///
/// This class is responsible for computing partial derivatives of the
/// following paramters:
///   - Cartesian state X-component
///   - Cartesian state Y-component
///   - Cartesian state Z-component
///   - Cartesian state dX-component
///   - Cartesian state dY-component
///   - Cartesian state dZ-component
///   - Gravitational body radius
///   - Gravitational body GM
///   - Gravitational body J2 term
///
class GravityAction : public Action
{
 public:
  GravityAction();
  GravityAction( const string name, const double radius, const double mu,
                 const double J2 );

 ~GravityAction() override;

  // Computes the acceleration due to this action and adds it to the
  // passed in vector "acceleration".
  void getAcceleration( vector< double > &acceleration,
                        const vector< double > &state ) const override;

  // Computes the partial derivative of the acceleration terms and owned
  // parameters
  void getPartials( vector< double > &partials, const vector< double > &state,
                    const vector< string >  &activeAgents ) override;
 private:
  string m_name;
  double m_radius;
  double m_mu;
  double m_J2;
  /// @todo need some way of identifying radius, mu, J2 for a
  /// particular gravitational body
  vector< string > m_agentsOwned = { "X", "Y", "Z", "dX", "dY", "dZ",
                                     "radius", "mu", "J2" };
  map< string, double > m_evaledPartials;

  double accJ2( const vector< double > &state, const char component ) const;

  double getAgentPartial( const string &top, const string &bottom );
  void evalPartials( const vector< double > &state );
};

#endif // EKF_GRAVITYACTION_HEADER_GUARD
