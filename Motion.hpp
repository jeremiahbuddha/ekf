#ifndef EKF_MOTION_INCLUDE_                                                    
#define EKF_MOTION_INCLUDE_  

#include <vector>
#include <Action.hpp>
#include <Agent.hpp>
#include <InitialCondition.hpp>

using namespace std;

class Motion {
   /*
   The motion class is responsible for building the equations of motion for a
   given body. 

   */
   public:

      Motion();
      //Motion( const InitialCondition &ic );
      ~Motion();

      // Add effect of action to motion
      void addAction( const Action &a ); 

      //// Get current time step
      //double getTime() const;
      //// Get value of state at current step
      //vector< double > getState() const;
      //// Get the partials of Motion wrt a group of Agents
      //vector< double > getPartials( const AgentGroup &a ) const; 

      // This method defines the Equations of Motion for the odeint integrator
      void operator() ( const vector< double > &x , 
                        vector< double > &dxdt , const double t  );

   private:

      //double m_time;
      //vector< double > m_state;                                                  
      vector< Action > m_actions;

      //void setInitialConditions( InitialCondition ic );
};

#endif // Include guard
