
#include <boost/numeric/odeint.hpp>                                              
#include <Motion.hpp>
#include <GravityAction.hpp>


int
main()
{
   // Set up Initial Conditions
   //string ic_name = "state";
   vector< double > state = { 757700.00, 5222607.00, 4851500.00,                                                            
                              2213.21, 4678.34, -5371.30 };
   //InitialCondition myIC( ic_name, ic_vals );

   // Set up a GravityAction
   GravityAction myGrav( "Earth", 6378136.3, 3.986004415E+14,    
                          7.29211585530066E-5, 1.082626925638815E-3 );

   // Set up a Motion
   Motion mySc;
   mySc.addAction( myGrav );

   // Integrate to t = 10
   size_t steps = boost::numeric::odeint::integrate( mySc, state, 0.0 , 20.0 , 0.1 );

   cout << "\n### State at time 20:\n";                                          
   cout << setprecision(18) << state[0] << "\n";                                     
   cout << state[1] << "\n";                                                         
   cout << state[2] << "\n";                                                         
   cout << state[3] << "\n";                                                         
   cout << state[4] << "\n";                                                         
   cout << state[5] << "\n";                                                         

   return 0;
}
