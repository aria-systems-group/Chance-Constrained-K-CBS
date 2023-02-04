#include "Conflict.h"
#include <ompl/base/State.h>

Conflict::Conflict(int agent1Idx, int agent2Idx, int timeStep):
    agent1Idx_(agent1Idx), agent2Idx_(agent2Idx), timeStep_(timeStep) {}

bool Conflict::operator==(const Conflict &other) const
{ 
    return (agent1Idx_ == other.agent1Idx_ && 
            agent2Idx_ == other.agent2Idx_ &&
            timeStep_ == other.timeStep_);
};
