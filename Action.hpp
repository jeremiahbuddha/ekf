#ifndef EKF_ACTION_INCLUDE_
#define EKF_ACTION_INCLUDE_

#include <vector>
#include <AgentGroup.hpp>

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

      // Computes the partial derivative of the acceleration terms and owned
      // parameters
      virtual void getPartials( vector < double > &partials,
                                const vector< double > &state,
                                const vector< string >  &activeAgents ) = 0;
      // Destructor
      virtual ~Action(){};

   protected:
     bool m_debug = false;

   private:


};

#endif // Include guard
