// -*- coding:utf-8; mode:c++; mode:auto-fill; fill-column:80; -*-

///
/// @file    AtmosphereAction.hpp
/// @brief   Computes state accelerations and partials due to the
///          interaction of an agent with a planetary atmosphere.
/// @author  Jonathon Smith <jonathon.j.smith@gmail.com>
/// @date    January 24, 2015
///

#pragma once
#ifndef EKF_ATMOSPHEREACTION_HEADER_GUARD
#define EKF_ATMOSPHEREACTION_HEADER_GUARD

// C++ Standard Library
#include <string>
#include <vector>
#include <map>

// ekf Library
#include <Action.hpp>

/// @brief Compute state accelerations and partial derivates due to
/// the interaction of an agent and planetary atmosphere.
///
/// This is a resource class for Motion, used to compute the
/// accelerations and partial derivatives wrt a planetary atmosphere
/// at a given Epoch / State.
///
/// This class is responsible for computing partial derivatives of the
/// following paramters:
///   - Cartesian state X, Y, Z, dX, dY, dZ components
///   - Exponential atmosphere referece height
///   - Exponential atmosphere reference density
///   - Exponential atmosphere step height
///   - Planetary rotation
///   - Agent body drag term
///
class AtmosphereAction : public Action
{
 public:
  AtmosphereAction();
  AtmosphereAction( const std::string name, double refHeight, double refDensity,
                    double stepHeight, double rotation, double bodyDragTerm );

 ~AtmosphereAction() override;

  // Computes the acceleration due to this action and adds it to
  // the passed in vector "acceleration".
  void getAcceleration( std::vector< double > &acceleration,
                        const std::vector< double > &state ) const override;

  // Computes the partial derivative of the acceleration terms and
  // owned parameters
  void getPartials( std::vector< double > &partials,
                    const std::vector< double > &state,
                    const std::vector< std::string >  &activeAgents ) override;
 private:
  std::string m_name;
  double m_refHeight;
  double m_refDensity;
  double m_stepHeight;
  double m_rotation;
  double m_bodyDragTerm;
  std::map< std::string, double > m_evaledPartials;

  /// @todo need some way of identifying h_ref, rho_ref, step, rot, Cd
  /// for a particular planetary atmosphere.
  std::vector< std::string > m_agentsOwned = { "X", "Y", "Z", "dX", "dY", "dZ",
                                             "h_ref", "rho_ref", "step", "rot",
                                             "Cd" };

  double adjustedDensity( const std::vector< double > state ) const;
  double adjustedVelocity( const std::vector< double > state ) const;

  double getAgentPartial( const std::string &top, const std::string &bottom );
  void evalPartials( const std::vector< double > &state );
};

#endif // EKF_ATMOSPHEREACTION_HEADER_GUARD
