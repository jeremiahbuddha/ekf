#ifndef EKF_GRAVITYACTION_INCLUDE_                                                      
#define EKF_GRAVITYACTION_INCLUDE_ 

#include <string>                                                                
#include <vector>
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
   private:
      string m_name;                                             
      double m_radius;
      double m_mu;
      double m_J2;

      double accJ2( const vector< double > &state, const char component ) const;

};

#endif // Inlcude guard
