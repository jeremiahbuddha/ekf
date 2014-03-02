
#include <iostream>
#include <cmath> 
#include <GravityAction.hpp>

using namespace std;

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
                                                                                 
// Computes the acceleration due to central body gravitation and the J2  
// perturbation.
void
GravityAction::
getAcceleration( 
   vector< double > &acceleration, 
   double t,            
   const vector< double > &state ) const
{ 
   double dist = sqrt( pow( state[0], 2 ) + pow( state[1], 2 ) + pow( state[2], 2 ) );
   acceleration[0] += -m_mu * state[0] / pow( dist, 3 ) * accJ2( state, 'x' ); 
   acceleration[1] += -m_mu * state[1] / pow( dist, 3 ) * accJ2( state, 'y' ); 
   acceleration[2] += -m_mu * state[2] / pow( dist, 3 ) * accJ2( state, 'z' ); 
}

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

