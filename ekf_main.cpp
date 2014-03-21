
#include <boost/numeric/odeint.hpp>                                              
#include <Motion.hpp>
#include <GravityAction.hpp>
#include <AtmosphereAction.hpp>

int
main()
{
   // Set up Motion
   vector< double > ic = { 757700., 5222607., 4851500., 
                           2213.21, 4678.34, -5371.30 };
   Motion mySc( ic, 1 );                                                                  

   // Set up a GravityAction and add to motion
   GravityAction myGrav( "Earth", 6378136.3, 3.986004415E+14,    
                          1.082626925638815E-3 );
   mySc.addAction( myGrav );

   // Set up a AtmosphereAction and add to motion                                   
   double bodyArea = 3.0; // m**2
   double bodyMass = 970.0; // kg
   double bodyCd = 2.0; 
   double bodyDragTerm = ( 1 / 2 ) * bodyCd * ( bodyArea / bodyMass );   
   AtmosphereAction myAtm( "Earth Atmosphere", 
                           7078136.3, 3.614E-13, 88667.0,                    
                          7.29211585530066E-5, bodyDragTerm );  
   mySc.addAction( myAtm );

   // Set up active agents
   vector< string > activeAgents = { 
      "mu", "J2", "Cd", 
      "X_1", "Y_1", "Z_1",
      "X_2", "Y_2", "Z_2",
      "X_3", "Y_3", "Z_3"
      };
   mySc.activateAgents( activeAgents );
 
   // Integrate to t = 10
   //mySc.stepTo( 10. );
   //mySc.printState( 5. );
   //mySc.printState( 10. );

   // Integrate to t = 200
   mySc.stepTo( 200. );
   mySc.printState( 200. );  

   return 0;
}
