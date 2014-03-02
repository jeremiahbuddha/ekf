#ifndef EKF_ODEINTHELPER_INCLUDE_                                                    
#define EKF_ODEINTHELPER_INCLUDE_  

#include <vector>
#include <Action.hpp>

using namespace std;

class OdeintHelper{                                                              
   public:                                                                       

      OdeintHelper( vector< const Action* > &actions );                                                            
      ~OdeintHelper();                                                           

      // Allows this class to be called by the odeint solver
      void operator() ( const vector< double > &x, vector< double > &dxdt, 
                        const double t );
                                                       
   private:                                                                      
      vector< const Action* > &m_actions;
};

#endif // Include guard
