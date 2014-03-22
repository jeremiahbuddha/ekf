
#include <iostream>
#include <OdeintHelper.hpp>

//=============================================================================  
//=============================================================================  
// CONSTRUCTORS / DESCTRUCTOR                

OdeintHelper::
OdeintHelper()
   : m_actions(),
     m_activeAgents()
{
}

OdeintHelper::
OdeintHelper( 
   vector< Action* > &actions,
   vector< string > &activeAgents )
   : m_actions( &actions ),
     m_activeAgents( &activeAgents ) 
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
   //cout << "\nAccumulating partials at time: " << t << endl; 

   // Accumulate accelerations from the different actions.                                                     
   vector< double > accel( 3, 0.0 );                                   
   for ( auto ap: *m_actions )                                              
   {                                                                       
      ap->getAcceleration( accel, x );                                    
   }                                                                       

   // Accumulate partials from the different actions.                                                     
   int numPartials = 6 * m_activeAgents->size();
   vector< double > partials( numPartials, 0.0 );                                         
   for ( auto ap: *m_actions )                                                    
   {                                                                             
      ap->getPartials( partials, x, *m_activeAgents );                                           
   }    

   // State elements                                                             
   dxdt[0] = x[3]; // X_dot                                                      
   dxdt[1] = x[4]; // Y_dot                                                      
   dxdt[2] = x[5]; // Z_dot                                                      
   dxdt[3] = accel[0]; // DX_dot                                                 
   dxdt[4] = accel[1]; // DY_dot                                                 
   dxdt[5] = accel[2]; // DY_dot                                           

   // State partials
   for ( int i = 6; i < numPartials; ++i )
   {
      dxdt[i] = partials[i - 6];  
   }
}

void
OdeintHelper::
howManyActions()
{
   cout << "There are " << m_actions->size() << " Actions in the helper" << endl; 
}     
