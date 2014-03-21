#ifndef EKF_MOTION_INCLUDE_                                                    
#define EKF_MOTION_INCLUDE_  

#include <vector>
#include <map>
#include <Eigen/Dense>                                                           
#include <Action.hpp>
#include <AgentGroup.hpp>
#include <OdeintHelper.hpp>

using namespace std;

class Motion {
   /*
   The motion class is responsible for defining the equations of motion for a
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
      void addAction( Action &a ); 
      // Activate agents for partials computations
      void activateAgents( const vector< string > agentNames );

      // Get current time step
      double getTime() const;
      // Get value of state at step t ( no arguments defaults to current time )
      vector< double > getState( double t ) const;
      // Get the partials of state at step t
      vector< double > getPartials( double t ) const; 

      // Print the current state to cout
      void printState( double t ) const;
      void printAllStates() const;

   private:

      double m_time;
      vector< double > m_state;
      vector< double > m_stm;
      vector< string > m_activeAgents;
      double m_step;                            
      vector< Action* > m_actions;
      OdeintHelper m_helper;  
      map< double, vector< double > > m_pastStates;

      void initializeStm( vector< string > &activeAgents );
};

#endif // Include guard
