
#include <Eigen/Dense>
#include <AgentGroup.hpp>

class Knowledge
{

   public:
      Knowledge();
      ~Knowledge();

      void step( double t );

   private:

      AgentGroup m_agents;
      Eigen::MatrixXd m_agentCovariance;
      


};
