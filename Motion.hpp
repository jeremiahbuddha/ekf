#ifndef EKF_MOTION_INCLUDE_                                                    
#define EKF_MOTION_INCLUDE_  

#include <vector>
#include <map>
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
      Motion( const vector< double > &ic, double step );
      ~Motion();

      // Step to time t
      void stepTo( double t );

      // Add effect of action to motion
      void addAction( const Action &a ); 

      // Get current time step
      double getTime() const;
      // Get value of state at step t ( no arguments defaults to current time )
      vector< double > getState( double t ) const;
      // Get the partials of Motion wrt a group of Agents
      vector< double > getPartials( double t, 
                                    const AgentGroup &a ) const; 

      // Print the current state to cout
      void printState( double t ) const;
      void printAllStates() const;

   private:

      double m_time;
      vector< double > m_state;
      double m_step;                            
      vector< const Action* > m_actions;
      OdeintHelper m_helper;  
      map< double, vector< double > > m_pastStates;
};

#endif // Include guard
