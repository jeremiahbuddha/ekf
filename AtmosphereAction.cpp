
#include <cmath>                                                                 
#include <AtmosphereAction.hpp>

using namespace std;

//=============================================================================  
//=============================================================================  
// CONSTRUCTORS / DESCTRUCTOR  

// Default Constructor                                                           
AtmosphereAction::
AtmosphereAction()
   : m_name(), 
     m_refHeight(),                                                          
     m_refDensity(),                                                        
     m_stepHeight(),                                                         
     m_rotation(),                                                           
     m_bodyDragTerm() 
{
}

// Constructor for standard planetary atmosphere
AtmosphereAction::                                                               
AtmosphereAction(
   const string name, 
   double refHeight, 
   double refDensity,  
   double stepHeight, 
   double rotation, 
   double bodyDragTerm )                                                               
   : m_name( name ),                                                                   
     m_refHeight( refHeight ),                                                                
     m_refDensity( refDensity ),                                                               
     m_stepHeight( stepHeight ),                                                               
     m_rotation( rotation ),                                                                 
     m_bodyDragTerm( bodyDragTerm )                                                              
{                                                                                
}   

// Default Destructor
AtmosphereAction::
~AtmosphereAction()
{
}

//=============================================================================  
//=============================================================================  
// PUBLIC MEMBERS   

// Computes the acceleration due to drag from a planetary body atmosphere. 
void                                                                             
AtmosphereAction::                                                               
getAcceleration(                                                                 
   vector< double > &acceleration,                                               
   const vector< double > &state ) const                                         
{                                                                                
   double dragPrefix =  - m_bodyDragTerm * adjustedDensity( state ) * adjustedVelocity( state );
                                                                                 
   acceleration[0] += dragPrefix * ( state[3] + state[1] * m_rotation );         
   acceleration[1] += dragPrefix * ( state[4] - state[0] * m_rotation );         
   acceleration[2] += dragPrefix * ( state[5] );                                 
}   

//=============================================================================  
//=============================================================================  
// PRIVATE MEMBERS   

// Get the atmospheric density at current state
double 
AtmosphereAction::
adjustedDensity( const vector< double > state ) const
{
   double dist = sqrt( pow( state[0], 2 ) + pow( state[1], 2 ) + pow( state[2], 2 ) );

   return m_refDensity * exp( - ( dist - m_refHeight ) / m_stepHeight );                                       

}                   

// Get the atmospheric relative velocity at current state
double
AtmosphereAction::
adjustedVelocity( const vector< double > state ) const
{
    return sqrt( pow( state[3] + state[1] * m_rotation, 2 ) +                                         
                 pow( state[4] - state[0] * m_rotation, 2 ) + 
                 pow( state[5], 2 ) );                               
}                                                                                 

