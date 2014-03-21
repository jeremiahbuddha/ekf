
#include <AgentGroup.hpp>

using namespace std;                                                             
                                                                                 
//=============================================================================  
//=============================================================================  
// CONSTRUCTORS / DESCTRUCTOR  

// Default constructor
AgentGroup::
AgentGroup()
   : m_agentNames(),                                           
     m_agentPartials()
{
}

// Construct with vector of agent names.
AgentGroup::
AgentGroup( const vector< string > agentNames )
   : m_agentNames( agentNames ),
     m_agentPartials()
{
   // Set partials matrix to correct size.
   int numAgents = m_agentNames.size();
   m_agentPartials.resize( numAgents, numAgents );
}

AgentGroup::                                                                     
~AgentGroup()                                                                     
{                                                                                
}    
