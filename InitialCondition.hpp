#ifndef EKF_INITIALCONDITION_INCLUDE_                                                      
#define EKF_INITIALCONDITION_INCLUDE_ 

#include <string>
#include <vector>
#include <map>

using namespace std;

class InitialCondition
{
   /*
   Used to seed initial numerical values to a Motion, Action or Agent.
   */
   public:
      InitialCondition();
      InitialCondition( const string &name, const vector< double > &vals );

      ~InitialCondition();

      // This should access a map that key:values name: initiail conditions
      // for that name.
      vector< double > getValues( const string name ); 

   private:
 
      void addCondition( const string name, const vector< double > vals );
      map< string, vector< double > > m_ics;

};

#endif // Include guard
