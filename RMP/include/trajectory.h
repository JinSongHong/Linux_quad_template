#include "globals.h"

class trajectory
{
private:
    
public:
    trajectory();
    ~trajectory();
    void traj_gen(StateModel_* state_model_FL, StateModel_* state_model_FR, StateModel_* state_model_RL, StateModel_* state_model_RR, double t);

};


