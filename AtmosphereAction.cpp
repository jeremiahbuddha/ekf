
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

// Computes the partial derivative of the acceleration terms with respect  
// to the state vector (x, y, z, dx, dy, dz) and adds it to the            
// passed in vector "partials".                                            
void 
AtmosphereAction::
getPartials( 
   vector< double > &partials,                              
   const vector< double > &state ) const
{
   // Condense variable names to make following equations more legible           
   double r = sqrt( pow( state[0], 2 ) + pow( state[1], 2 ) + pow( state[2], 2 ) );
   double X = state[0];                                                          
   double Y = state[1];                                                          
   double Z = state[2];                                                          
   double dX = state[3];                                                         
   double dY = state[4];                                                         
   double dZ = state[5];
   double step = m_stepHeight;                                                         
   double rot =  m_rotation;
   double rho = adjustedDensity( state );
   double vel = adjustedVelocity( state );                                                   
   double Cd = m_bodyDragTerm;
   vector< double > &p = partials;                                               
                                                                                 
   // Partials of acceleration X component wrt state.                            
   // d( accX ) / d(x)                                                           
   p[0] += ( Cd * rho * vel * X * ( dX + rot * Y ) / ( r * step ) +    
            -Cd * rho * ( -rot * dY + pow( rot, 2 ) * X ) * ( dX + rot * Y ) / vel );
   // d( accX ) / d(y)                                                           
   p[1] += ( Cd * rho * vel * Y * ( dX + rot * Y ) / ( r * step ) +    
            -Cd * rho * ( rot * dX + pow( rot, 2 ) * Y ) * ( dX + rot * Y ) / vel + 
            -Cd * rho * vel * rot ); 
   // d( accX ) / d(z)                                                           
   p[2] += Cd * rho * vel * Z * ( dX + rot * Y ) / ( r * step );
   // d( accX ) / d(dx)                                                          
   p[3] += -Cd * rho * pow( dX + rot * Y, 2 ) / vel - Cd * rho * vel ;                                                                    
   // d( accX ) / d(dy)                                                          
   p[4] += -Cd * rho * ( dY - rot * X ) * ( dX + rot * Y ) / vel;                                                                    
   // d( accX ) / d(dz)                                                          
   p[5] += -Cd * rho * dZ * ( dX + rot * Y ) / vel; 

   // Partials of acceleration Y component wrt state.                            
   // d( accY ) / d(x)                                                           
   p[6] += ( Cd * rho * vel * X * ( dY - rot * X ) / ( r * step ) +     
            -Cd * rho * ( pow( rot, 2 ) * X - rot * dY ) * ( dY - rot * X ) / vel +
             Cd * rho * vel * rot );
   // d( accY ) / d(y)                                                           
   p[7] += ( Cd * rho * vel * Y * ( dY - rot * X ) / ( r * step) +     
            -Cd * rho * ( rot * dX + pow( rot, 2 ) * Y ) * ( dY - rot * X ) / vel );
   // d( accY ) / d(z)                                                           
   p[8] += Cd * rho * vel * Z * ( dY - rot * X ) / ( r * step );
   // d( accY ) / d(dx)                                                          
   p[9] += -Cd * rho * ( dY - rot * X ) * ( dX + rot * Y ) / vel;                                                                    
   // d( accY ) / d(dy)                                                          
   p[10] += -Cd * rho * pow( dY - rot * X, 2 ) / vel - Cd * rho * vel;                                                                   
   // d( accY ) / d(dz)                                                          
   p[11] += -Cd * rho * dZ * ( dY - rot * X ) / vel;                                                                   
                                                                                 
   // Partials of acceleration Z component wrt state.                            
   // d( accZ ) / d(x)                                                           
   p[12] += ( Cd * rho * vel * dZ * X / (r * step) +                    
             -Cd * rho * dZ * ( pow( rot, 2 ) * X - rot * dY ) / vel );
   // d( accZ ) / d(y)                                                           
   p[13] += ( Cd * rho * vel * dZ * Y / ( r * step ) +                    
             -Cd * rho * dZ * ( rot * dX + pow( rot, 2 ) * Y) / vel );
   // d( accZ ) / d(z)                                                           
   p[14] += Cd * rho * vel * Z * dZ / ( r * step );
   // d( accZ ) / d(dx)                                                          
   p[15] += -Cd * rho * dZ * ( dX + rot * Y ) / vel;                                                
   // d( accZ ) / d(dy)                                                          
   p[16] += -Cd * rho * dZ * ( dY - rot * X ) / vel;                       
   // d( accZ ) / d(dz)                                                          
   p[17] += ( -Cd * rho * pow( dZ, 2 ) / vel ) + ( -Cd * rho * vel ); 

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

