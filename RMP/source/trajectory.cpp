#include "trajectory.h"

trajectory::trajectory()
{
}

trajectory::~trajectory()
{
}

void trajectory::traj_gen(StateModel_* state_model_FL, StateModel_* state_model_FR, StateModel_* state_model_RL, StateModel_* state_model_RR, double t)
{




    if(t > 0)
    {
    state_model_RR->vel_ref[0] = 5*sin(t);
    state_model_RR->vel_ref[1] = 5*sin(t);
    state_model_RR->vel_ref[2] = 5*sin(t);
    }
    
}

