#ifndef EKF_AGENTGROUP_INCLUDE_                                                      
#define EKF_AGENTGROUP_INCLUDE_ 

#include <vector>
#include <Eigen/Dense> 

using namespace std;

class AgentGroup 
{
   /* 
   An AgentGroup holds the names of nondynamic parameters, and a matrix to 
   collect their partial derivatives. 
   */
   public:
      AgentGroup();
      AgentGroup( const vector< string > agentNames );
      ~AgentGroup();

   private:
      vector< string > m_agentNames;
      Eigen::MatrixXd m_agentPartials;       

};

#endif // Include guard
