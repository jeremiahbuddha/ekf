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
      // Constructor
      Action(){};

      // Computes the acceleration due to this action and adds it to the
      // passed in vector "acceleration".
      virtual void getAcceleration( vector< double > &acceleration,
                                     const vector< double > &state ) const = 0; 
      // Destructor
      virtual ~Action(){};
   private:

};

#endif // Include guard
