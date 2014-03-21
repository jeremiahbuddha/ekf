#ifndef EKF_GRAVITYACTION_INCLUDE_                                                      
#define EKF_GRAVITYACTION_INCLUDE_ 

#include <string>                                                                
#include <vector>
#include <map>
#include <Action.hpp>

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
      void getPartials( vector< double > &partials, 
                        const vector< double > &state,                                                
                        const vector< string >  &activeAgents ) override; 
   private:
      string m_name;                                             
      double m_radius;
      double m_mu;
      double m_J2;
      vector< string > m_agentsOwned = { "X", "Y", "Z", "dX", "dY", "dZ", 
                                         "radius", "mu", "J2" };
      map< string, double > m_evaledPartials;

      double accJ2( const vector< double > &state, const char component ) const;

      double getAgentPartial( const string &top, const string &bottom ); 
      void evalPartials( const vector< double > &state );                                          
};

#endif // Inlcude guard
