
#include <iostream>
#include <"Force.hpp">
#include <"Agent.hpp">
#include <"InitialCondition.hpp">

class Motion {
   /*
   The motion class is responsible for building the equations of motion for a
   given body, and 

   */
   public:

      Motion();

      Motion( const InitialCondition ic );

      ~Motion();

      void AddForce( const Force f ); // add effect of force to motion

      void getAcceleration( const Time t ) const; // get acceleration of motion at time t

      void getPartials( const AgentGroup a ); // get partials of motion with respect to a group of Agents

   private:

      void setInitialConditions( const InitialCondition ic )

};
