#ifndef EKF_ATMOSPHEREACTION_INCLUDE_                                                      
#define EKF_ATMOSPHEREACTION_INCLUDE_   

#include <string>
#include <vector>
#include <map>
#include <Action.hpp>

class AtmosphereAction : public Action
{

   public:
      AtmosphereAction();
      AtmosphereAction( const string name, double refHeight, double refDensity,
                        double stepHeight, double rotation, 
                        double bodyDragTerm ); 
 
      ~AtmosphereAction() override;
                                                                                 
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
      double m_refHeight;
      double m_refDensity;
      double m_stepHeight;
      double m_rotation;
      double m_bodyDragTerm;
      map< string, double > m_evaledPartials;                                    
      
      double adjustedDensity( const vector< double > state ) const;
      double adjustedVelocity( const vector< double > state ) const;
                                                                                 
      double getAgentPartial( const string &top, const string &bottom );          
      void evalPartials( const vector< double > &state );  

};

#endif // Include guard
