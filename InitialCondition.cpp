
#include <InitialCondition.hpp>


// Default Constructor
InitialCondition::
InitialCondition()
   : m_ics()
{
}

// Construct with single set of ICs
InitialCondition::
InitialCondition(
   const string &name,
   const vector< double > &vals )
   : m_ics()
{
   // Add initial conditions 
   m_ics.insert( make_pair( name, vals ) );
}

// Default Destructor
InitialCondition::
~InitialCondition()
{
}

// Get a set of ICs by name
vector< double > 
InitialCondition::
getValues( 
   const string name )
{
   return m_ics[name];
}

void
InitialCondition::
addCondition(
   const string name, 
   const vector< double > vals )
{
   // Add initial conditions                                                     
   m_ics.insert( make_pair( name, vals ) );
}
