#ifndef EKF_ODEINTHELPER_INCLUDE_                                                    
#define EKF_ODEINTHELPER_INCLUDE_  

#include <vector>
#include <Eigen/Dense>                                                           
#include <Action.hpp>

using namespace std;

class OdeintHelper{                                                              
   public:                                                                       

      OdeintHelper();
      OdeintHelper( vector< Action* > &actions, 
                    vector< string > &activeAgents );                                                            
      ~OdeintHelper();                                                           

      // Allows this class to be called by the odeint solver
      void operator() ( const vector< double > &x, vector< double > &dxdt, 
                        const double t );
                                                       
   private:                                                                      
      vector< Action* > m_actions;
      vector< string > m_activeAgents; 
};

#endif // Include guard
