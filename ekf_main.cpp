
#include <boost/numeric/odeint.hpp>                                              
#include <Motion.hpp>
#include <GravityAction.hpp>

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

   // Integrate to t = 10
   mySc.stepTo( 10. );
   mySc.printState( 5. );
   mySc.printState( 10. );

   // Integrate to t = 20
   mySc.stepTo( 20. );
   mySc.printState( 15. );                                                        
   mySc.printState( 20. );  

   return 0;
}
