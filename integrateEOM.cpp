
#include <iostream>
#include <vector>
#include <cmath>
#include <boost/numeric/odeint.hpp>

using namespace std;

// Earth constants
double dTheta = 7.29211585530066E-5;
double J2 = 1.082626925638815E-3;
double mu = 3.986004415E+14; // m**3 / s**2
double R = 6378136.3; // m

// Drag Params
double Cd = 2.0; // unitless
double A = 3.0; // m**2
double m = 970.0; // kg
double rho_0 = 3.614E-13; // kg / m**3
double ref_r = 7078136.3; // m
double h_step = 88667.0; // m
double drag_C = ( 1.0 / 2.0 ) * Cd * ( A / m );


// Return the magnintude of the first three components of the state vector
double
r( const vector< double > &x )
{
   return sqrt( pow( x[0], 2 ) +
                pow( x[1], 2 ) +
                pow( x[2], 2 ) );
}

// Evaluate the acceleration due to J2
double
acc_j2( const vector< double > &x, const string &comp )
{
   // This function augments the two-body EOMs with a J2 term.
   if ( comp.compare( "x" ) || comp.compare( "y" ) )
   {
      return ( 1.0 - 1.5 * J2 * pow( ( R / r( x ) ), 2.0 ) * 
               ( 5 * pow( ( x[2] / r( x )), 2) - 1 ) );
   }
   else if ( comp.compare( "z" ) )
   {
      return ( 1.0 - 1.5 * J2 * pow( ( R / r( x ) ), 2 ) * 
               ( 5 * pow( ( x[2] / r( x ) ), 2 ) - 3 ) );
   }
}

// Evaluate the acceleration due to Drag
double
rho_A( const vector< double > &x )
{
   return rho_0 * exp( - ( r(x) - ref_r ) / h_step );
}

double
V_A( const vector< double > &x )
{
   return sqrt( pow( ( x[3] + x[1] * dTheta ), 2 ) +
                pow( x[4] - x[0] * dTheta, 2 ) + pow( x[5] , 2 ) );
}

double 
drag_prefix( const vector< double > &x )
{                                                          
   return -drag_C * rho_A( x ) * V_A( x );
}

double
acc_drag( const vector< double > x, const string comp )
{
   // This function augments the two-body EOMs with a drag term.
   if ( comp.compare( "x" ) )
   {
      return drag_prefix( x ) * ( x[3] + x[1] * dTheta );
   }
   else if ( comp.compare( "y" ) )
   {
      return drag_prefix( x ) * ( x[4] - x[0] * dTheta );
   }
   else if ( comp.compare( "z" ) )
   {
      return drag_prefix( x ) * x[5];
   }
}

/* The rhs of x' = f(x) */
void eom( const vector< double > &x , vector< double > &dxdt , const double t )
{
   // State elements
   dxdt[0] = x[3]; // X_dot
   dxdt[1] = x[4]; // Y_dot
   dxdt[2] = x[5]; // Z_dot
   dxdt[3] = -mu * x[0] / pow( r( x ), 3 ) * acc_j2( x, "x" ) + acc_drag( x, "x" ); // DX_dot
   dxdt[4] = -mu * x[1] / pow( r( x ), 3 ) * acc_j2( x, "y" ) + acc_drag( x, "y" ); // DY_dot
   dxdt[5] = -mu * x[2] / pow( r( x ), 3 ) * acc_j2( x, "z" ) + acc_drag( x, "z" ); // DY_dot
   dxdt[6] = 0.0; // mu_dot
   dxdt[7] = 0.0; // J2_dot
   dxdt[8] = 0.0; // Cd_dot
   dxdt[9] = 0.0; // X1
   dxdt[10] = 0.0; // Y1
   dxdt[11] = 0.0; // Z1
   dxdt[12] = 0.0; // X2
   dxdt[13] = 0.0; // Y2
   dxdt[14] = 0.0; // Z2
   dxdt[15] = 0.0; // X3
   dxdt[16] = 0.0; // Y3
   dxdt[17] = 0.0; // Z3
}

int 
main()
{
   cout << "\n### Starting integration!\n";

   // Define initial conditions
   vector < double > stn101{ -5127510.0,-3794160.0, 0.0 };
   vector < double > stn337{ 93860910.0, 3238490.0, 3898094.0 };
   vector < double > stn394{   549505.0,-1380872.0, 6182197.0 };

   vector < double > X{
      757700.0,  // X
      5222607.0, // Y
      4851500.0, // Z
      2213.21,   // dX
      4678.34,   // dY 
      -5371.30,   // dZ
      mu,        // mu
      J2,        // J2
      Cd,        // Cd
      stn101[0], // X_1
      stn101[1], // Y_1
      stn101[2], // Z_1
      stn337[0], // X_1
      stn337[1], // Y_1
      stn337[2], // Z_1
      stn394[0], // X_1
      stn394[1], // Y_1
      stn394[2], // Z_1
      };

   cout << "\n### Value at time 0:\n";                                           
   cout << X[0];   

   size_t steps = boost::numeric::odeint::integrate( eom, X , 0.0 , 20.0 , 0.1 );

   cout << "\n### Value at time 10:\n";
   cout << setprecision(18) << X[0] << "\n";
   cout << X[1] << "\n";                                                         
   cout << X[2] << "\n";                                                         
   cout << X[3] << "\n";                                                         
   cout << X[4] << "\n";                                                         
   cout << X[5] << "\n";                                                         
   cout << X[6] << "\n";                                                         
   cout << X[7] << "\n";                                                         
   cout << X[8] << "\n";                                                         
   cout << X[9] << "\n";                                                         
   
   return 0;
}
