
#include <iostream>
#include <cmath> 
#include <GravityAction.hpp>

using namespace std;

//=============================================================================  
//=============================================================================  
// CONSTRUCTORS / DESCTRUCTOR 

// Default Constructor 
GravityAction::                                                                  
GravityAction() 
   : m_name(),                                                             
     m_radius(),                                                         
     m_mu(),                                                                 
     m_J2()                                                                  
{                                                                                
}  

// Constructor for standard solar system central body
GravityAction::
GravityAction(
   const string name, 
   const double radius, 
   const double mu,    
   const double J2 )
   : m_name( name ), 
     m_radius( radius ), 
     m_mu( mu ),
     m_J2( J2 )
{
} 

// Destructor
GravityAction::                                                                                 
~GravityAction()
{
}                                                 

//=============================================================================  
//=============================================================================  
// PUBLIC MEMBERS 
                                                                                 
// Computes the acceleration due to central body gravitation and the J2  
// perturbation.
void
GravityAction::
getAcceleration( 
   vector< double > &acceleration, 
   const vector< double > &state ) const
{ 
   double dist = sqrt( pow( state[0], 2 ) + pow( state[1], 2 ) + pow( state[2], 2 ) );
   acceleration[0] += -m_mu * state[0] / pow( dist, 3 ) * accJ2( state, 'x' ); 
   acceleration[1] += -m_mu * state[1] / pow( dist, 3 ) * accJ2( state, 'y' ); 
   acceleration[2] += -m_mu * state[2] / pow( dist, 3 ) * accJ2( state, 'z' ); 
}

// Computes the partial derivatives of the acceleration due to central body 
// gravitation and the J2 perturbation with respect to Motion parameters 
// x, y, z, dx, dy, dz                                                                 
void                                                                             
GravityAction::                                                                  
getPartials(                                                                 
   vector< double > &partials,                                               
   const vector< double > &state ) const                                         
{                                                                                
   // Condense variable names to make following equations more legible
   double r = sqrt( pow( state[0], 2 ) + pow( state[1], 2 ) + pow( state[2], 2 ) );
   double R = m_radius;                                                              
   double mu = m_mu;
   double J2 = m_J2;                                                                     
   double X = state[0];
   double Y = state[1];                                                          
   double Z = state[2];                                                          
   double dX = state[3];                                                          
   double dY = state[4];                                                          
   double dZ = state[5];                                                          
   double r3 = pow( r, 3 );
   double r5 = pow( r, 5 );
   double R_r2 = pow( R / r, 2 ); 
   double Z_r2 = pow( Z / r, 2 );
   vector< double > &p = partials;

   // Partials of acceleration X component wrt state.
   // d( accX ) / d(x)
   p[0] += ( - mu / r3 * ( 1 - ( 3 / 2 ) * J2 * R_r2 * ( 5 * Z_r2 - 1.) ) + 
             3 * mu * pow( X, 2 ) / r5 * ( 1 - ( 5 / 2 ) * J2 * R_r2 * ( 7 * Z_r2 - 1 ) ) );
   // d( accX ) / d(y)
   p[1] += 3 * mu * X * Y / r5 * ( 1 - ( 5 / 2 ) * J2 * R_r2 * ( 7 * Z_r2 - 1 ) ); 
   // d( accX ) / d(z)
   p[2] += 3 * mu * X * Z / r5 * ( 1 - ( 5 / 2 ) * J2 * R_r2 * ( 7 * Z_r2 - 3 ) );
   // d( accX ) / d(dx) 
   p[3] += 0; 
   // d( accX ) / d(dy) 
   p[4] += 0; 
   // d( accX ) / d(dz) 
   p[5] += 0;

   // Partials of acceleration Y component wrt state.                            
   // d( accY ) / d(x)
   p[6] += 3 * mu * X * Y / r5 * ( 1 - ( 5  / 2 ) * J2 * R_r2 * ( 7 * Z_r2 - 1 ) ); 
   // d( accY ) / d(y)
   p[7] += ( - mu / r3 * ( 1 - ( 3 / 2 ) * J2 * R_r2 * ( 5 * Z_r2 - 1 ) ) + 
             3 * mu * pow( Y, 2 ) / r5 * ( 1 - ( 5 / 2 ) * J2 * R_r2 * ( 7 * Z_r2 - 1 ) ) ); 
   // d( accY ) / d(z)
   p[8] += 3 * mu * Y * Z / r5 * ( 1 - ( 5 / 2 ) * J2 * R_r2 * ( 7 * Z_r2 - 3 ) );
   // d( accY ) / d(dx)
   p[9] += 0;
   // d( accY ) / d(dy)
   p[10] += 0;
   // d( accY ) / d(dz)
   p[11] += 0;

   // Partials of acceleration Z component wrt state.                            
   // d( accZ ) / d(x)
   p[12] += 3 * mu * X * Z / r5 * ( 1 - ( 5 / 2 ) * J2 * R_r2 * ( 7 * Z_r2 - 3 ) );
   // d( accZ ) / d(y)
   p[13] += 3 * mu * Y * Z / r5 * ( 1 - ( 5 / 2 ) * J2 * R_r2 * ( 7 * Z_r2 - 3 ) );
   // d( accZ ) / d(z)
   p[14] += ( - mu / r3 * ( 1 - ( 3 / 2 ) * J2 * R_r2 * ( 5 * Z_r2 - 3 ) ) + 
              3 * mu * pow( Z, 2 ) / r5 * ( 1 - ( 5 / 2 ) * J2 * R_r2 * ( 7 * Z_r2 - 5 ) ) );
   // d( accZ ) / d(dx)
   p[15] += 0;
   // d( accZ ) / d(dy)
   p[16] += 0;
   // d( accZ ) / d(dz)
   p[17] += 0;
}  

//=============================================================================  
//=============================================================================  
// PRIVATE MEMBERS   

// Computes the J2 gravitational perturbation, by state component.
double
GravityAction::
accJ2(
   const vector< double > &state, 
   const char component ) const
{
   double dist = sqrt( pow( state[0], 2 ) + pow( state[1], 2 ) + pow( state[2], 2 ) );  

   // This function augments the two-body EOMs with a J2 term.                   
   if ( ( component == 'x' ) || ( component == 'y' ) )                             
   {                                                                             
      return ( 1.0 - 1.5 * m_J2 * pow( ( m_radius / dist ), 2.0 ) *                     
               ( 5 * pow( ( state[2] / dist ), 2) - 1 ) );                          
   }                                                                             
   else if ( component == 'z' )                                                
   {                                                                             
      return ( 1.0 - 1.5 * m_J2 * pow( ( m_radius / dist ), 2 ) *                       
               ( 5 * pow( ( state[2] / dist ), 2 ) - 3 ) );                        
   }    
}

