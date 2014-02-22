#ifndef EKF_MOTION_INCLUDE_                                                    
#define EKF_MOTION_INCLUDE_  

#include <vector>
#include <Time.hpp>
#include <Action.hpp>
#include <Agent.hpp>
#include <InitialCondition.hpp>

using namespace std;

class Motion {
   /*
   The motion class is responsible for building the equations of motion for a
   given body, and 

   */
   public:

      Motion();

      Motion( const InitialCondition ic );

      ~Motion();

      void addAction( const Action a ); // add effect of action to motion

      void getAcceleration( const Time t ) const; // get acceleration of motion at time t

      void getPartials( const AgentGroup a ); // get partials of motion with respect to a group of Agents

   private:

      void setInitialConditions( const InitialCondition ic );
      vector< double > m_state;
};

#endif // Include guard
