
#include <iostream>
#include <OdeintHelper.hpp>

//=============================================================================  
//=============================================================================  
// CONSTRUCTORS / DESCTRUCTOR                

OdeintHelper::
OdeintHelper( vector< const Action* > &actions )
   : m_actions( actions )
{
}

OdeintHelper::
~OdeintHelper()
{
}

//=============================================================================  
//=============================================================================  
// PUBLIC MEMBERS       

// This method defines the equations of motion for the odeint integrator   
void 
OdeintHelper::
operator() (                                                          
   const vector< double > &x ,                                             
   vector< double > &dxdt ,                                                
   const double t  )                                                       
{                                                                          
   vector< double > accel = { 0, 0, 0 };                                   
   // Accumulate accelerations from the different actions.                                                     
   for ( auto ap: m_actions )                                              
   {                                                                       
      ap->getAcceleration( accel, x );                                    
   }                                                                       
   // State elements                                                             
   dxdt[0] = x[3]; // X_dot                                                      
   dxdt[1] = x[4]; // Y_dot                                                      
   dxdt[2] = x[5]; // Z_dot                                                      
   dxdt[3] = accel[0]; // DX_dot                                                 
   dxdt[4] = accel[1]; // DY_dot                                                 
   dxdt[5] = accel[2]; // DY_dot                                           
}     
