#ifndef EKF_ACTION_INCLUDE_                                                      
#define EKF_ACTION_INCLUDE_ 

#include <vector>

using namespace std;

class Action
{
   /*
   The Action class defines a force capable of effecting the evolution of a 
   Motion object.
   */
   public:
      Action();
      ~Action();

      // Computes the acceleration due to this action and adds it to the
      // passed in vector "acceleration".
      virtual void get_acceleration( vector< double > &acceleration,
                                     const Time &t, 
                                     const vector< double > &state ); 
                               
   private:

};

#endif // Include guard
