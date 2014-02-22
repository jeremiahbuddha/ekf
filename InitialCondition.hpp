#ifndef EKF_INITIALCONDITION_INCLUDE_                                                      
#define EKF_INITIALCONDITION_INCLUDE_ 

class InitialCondition
{
   /*
   Used to seed initial numerical values to a Motion, Action or Agent.
   */
   public:
      InitialCondition();
      ~InitialCondition();

      // This should access a map that key:values name: initiail conditions
      // for that name.
      vector< double > getValues( const string name ) const; 

   private:

};

#endif // Include guard
