#ifndef EKF_MOTION_INCLUDE_                                                    
#define EKF_MOTION_INCLUDE_  

#include <vector>
#include <Action.hpp>
#include <Agent.hpp>
#include <OdeintHelper.hpp>

using namespace std;

class Motion {
   /*
   The motion class is responsible for definingthe equations of motion for a
   given body, integrating it to any given time, and storing the history of
   the bodies motion since it's ICs.
   */
   public:
      Motion();
      Motion( const vector< double > &ic );
      ~Motion();

      // Step to time t
      void step( double t );

      // Add effect of action to motion
      void addAction( const Action &a ); 

      // Get current time step
      double getTime() const;
      // Get value of state at current step
      vector< double > getState() const;
      // Get the partials of Motion wrt a group of Agents
      vector< double > getPartials( const AgentGroup &a ) const; 

      // Print the current state to cout
      void printState() const;

   private:

      double m_time;
      vector< double > m_state;                                                  
      vector< const Action* > m_actions;
      OdeintHelper m_helper;  
 
};

#endif // Include guard
