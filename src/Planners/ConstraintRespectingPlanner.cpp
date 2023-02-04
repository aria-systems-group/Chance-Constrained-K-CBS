#include "Planners/ConstraintRespectingPlanner.h"


void oc::PlannerInputStates::clear()
{
    if (tempState_ != nullptr)
    {
        si_->freeState(tempState_);
        tempState_ = nullptr;
    }
    addedStartStates_ = 0;
    sampledGoalsCount_ = 0;
    pdef_.reset();
    si_ = nullptr;
}
  
void oc::PlannerInputStates::restart()
{
    addedStartStates_ = 0;
    sampledGoalsCount_ = 0;
}
  
bool oc::PlannerInputStates::update()
{
    if (planner_ == nullptr)
        throw Exception("No planner set for PlannerInputStates");
    return use(planner_->getProblemDefinition());
}
  
void oc::PlannerInputStates::checkValidity() const
{
    std::string error;
  
    if (pdef_ == nullptr)
        error = "Problem definition not specified";
    else
    {
        if (pdef_->getStartStateCount() <= 0)
            error = "No start states specified";
        else if (!pdef_->getGoal())
            error = "No goal specified";
    }
  
    if (!error.empty())
    {
        if (planner_ != nullptr)
            throw Exception(planner_->getName(), error);
        else
            throw Exception(error);
    }
}
  
bool oc::PlannerInputStates::use(const ob::ProblemDefinitionPtr &pdef)
{
    if (pdef && pdef_ != pdef)
    {
        clear();
        pdef_ = pdef;
        si_ = pdef->getSpaceInformation().get();
        return true;
    }
    return false;
}
  
const ob::State *oc::PlannerInputStates::nextStart()
{
    if (pdef_ == nullptr || si_ == nullptr)
    {
       std::string error = "Missing space information or problem definition";
       if (planner_ != nullptr)
           throw Exception(planner_->getName(), error);
       else
           throw Exception(error);
    }
  
    while (addedStartStates_ < pdef_->getStartStateCount())
    {
        const base::State *st = pdef_->getStartState(addedStartStates_);
        addedStartStates_++;
        bool bounds = si_->satisfiesBounds(st);
        bool valid = bounds ? si_->isValid(st) : false;
        if (bounds && valid)
            return st;
  
        OMPL_WARN("%s: Skipping invalid start state (invalid %s)",
                  planner_ ? planner_->getName().c_str() : "PlannerInputStates", bounds ? "state" : "bounds");
        std::stringstream ss;
        si_->printState(st, ss);
        OMPL_DEBUG("%s: Discarded start state %s", planner_ ? planner_->getName().c_str() : "PlannerInputStates",
                   ss.str().c_str());
    }
    return nullptr;
 }
  
const ob::State *oc::PlannerInputStates::nextGoal()
{
    // This initialization is safe since we are in a non-const function anyway.
    static ob::PlannerTerminationCondition ptc = ob::plannerAlwaysTerminatingCondition();
    return nextGoal(ptc);
}
  
const ob::State *oc::PlannerInputStates::nextGoal(const ob::PlannerTerminationCondition &ptc)
{
    if (pdef_ == nullptr || si_ == nullptr)
    {
        std::string error = "Missing space information or problem definition";
        if (planner_ != nullptr)
            throw Exception(planner_->getName(), error);
        else
            throw Exception(error);
    }
  
    if (pdef_->getGoal() != nullptr)
    {
        const ob::GoalSampleableRegion *goal =
            pdef_->getGoal()->hasType(ob::GOAL_SAMPLEABLE_REGION) ? pdef_->getGoal()->as<ob::GoalSampleableRegion>() : nullptr;
  
        if (goal != nullptr)
        {
            time::point start_wait;
            bool first = true;
            bool attempt = true;
            while (attempt)
            {
                attempt = false;
  
                if (sampledGoalsCount_ < goal->maxSampleCount() && goal->canSample())
                {
                    if (tempState_ == nullptr)
                        tempState_ = si_->allocState();
                    do
                    {
                        goal->sampleGoal(tempState_);
                        sampledGoalsCount_++;
                        bool bounds = si_->satisfiesBounds(tempState_);
                        bool valid = bounds ? si_->isValid(tempState_) : false;
                        if (bounds && valid)
                        {
                            if (!first)  // if we waited, show how long
                            {
                                OMPL_DEBUG("%s: Waited %lf seconds for the first goal sample.",
                                           planner_ ? planner_->getName().c_str() : "PlannerInputStates",
                                           time::seconds(time::now() - start_wait));
                            }
                            return tempState_;
                        }
  
                        OMPL_WARN("%s: Skipping invalid goal state (invalid %s)",
                                  planner_ ? planner_->getName().c_str() : "PlannerInputStates",
                                  bounds ? "state" : "bounds");
                        std::stringstream ss;
                        si_->printState(tempState_, ss);
                        OMPL_DEBUG("%s: Discarded goal state:\n%s",
                                   planner_ ? planner_->getName().c_str() : "PlannerInputStates", ss.str().c_str());
                    } while (!ptc && sampledGoalsCount_ < goal->maxSampleCount() && goal->canSample());
                }
                if (goal->couldSample() && !ptc)
                {
                    if (first)
                    {
                        first = false;
                        start_wait = time::now();
                        OMPL_DEBUG("%s: Waiting for goal region samples ...",
                                   planner_ ? planner_->getName().c_str() : "PlannerInputStates");
                    }
                    std::this_thread::sleep_for(time::seconds(0.01));
                    attempt = !ptc;
                }
            }
        }
    }
  
    return nullptr;
}
  
bool oc::PlannerInputStates::haveMoreStartStates() const
{
    if (pdef_ != nullptr)
        return addedStartStates_ < pdef_->getStartStateCount();
    return false;
}
  
bool oc::PlannerInputStates::haveMoreGoalStates() const
{
    if ((pdef_ != nullptr) && pdef_->getGoal())
        if (pdef_->getGoal()->hasType(ob::GOAL_SAMPLEABLE_REGION))
            return sampledGoalsCount_ < pdef_->getGoal()->as<ob::GoalSampleableRegion>()->maxSampleCount();
    return false;
}

ConstraintRespectingPlanner::ConstraintRespectingPlanner(ob::SpaceInformationPtr si, std::string name): 
    si_(std::move(si)), pis_(this), name_(std::move(name)), setup_(false)
{
    if (!si_)
        throw ompl::Exception(name_, "Invalid space information instance for planner");
}
  
const ob::PlannerSpecs &ConstraintRespectingPlanner::getSpecs() const
{
    return specs_;
}
  
const std::string &ConstraintRespectingPlanner::getName() const
{
    return name_;
}
  
void ConstraintRespectingPlanner::setName(const std::string &name)
{
    name_ = name;
}
  
const ob::SpaceInformationPtr &ConstraintRespectingPlanner::getSpaceInformation() const
{
    return si_;
}
  
const ob::ProblemDefinitionPtr &ConstraintRespectingPlanner::getProblemDefinition() const
{
    return pdef_;
}
  
ob::ProblemDefinitionPtr &ConstraintRespectingPlanner::getProblemDefinition()
{
    return pdef_;
}
  
void ConstraintRespectingPlanner::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
    pdef_ = pdef;
    pis_.update();
}
  
const oc::PlannerInputStates &ConstraintRespectingPlanner::getPlannerInputStates() const
{
    return pis_;
}
  
void ConstraintRespectingPlanner::setup()
{
    if (!si_->isSetup())
    {
        OMPL_INFORM("%s: Space information setup was not yet called. Calling now.", getName().c_str());
        si_->setup();
    }
 
    if (setup_)
        OMPL_WARN("%s: Planner setup called multiple times", getName().c_str());
    else
        setup_ = true;
}
  
void ConstraintRespectingPlanner::checkValidity()
{
    if (!isSetup())
        setup();
    pis_.checkValidity();
}
  
bool ConstraintRespectingPlanner::isSetup() const
{
    return setup_;
}
  
void ConstraintRespectingPlanner::clear()
{
    pis_.clear();
    pis_.update();
}
  
void ConstraintRespectingPlanner::clearQuery()
{
    clear();
}
  
void ConstraintRespectingPlanner::getPlannerData(ob::PlannerData &data) const
{
    for (const auto &plannerProgressProperty : plannerProgressProperties_)
        data.properties[plannerProgressProperty.first] = plannerProgressProperty.second();
}
  
ob::PlannerStatus ConstraintRespectingPlanner::solve(const ob::PlannerTerminationConditionFn &ptc, double checkInterval)
{
    return solve(ob::PlannerTerminationCondition(ptc, checkInterval));
}
  
ob::PlannerStatus ConstraintRespectingPlanner::solve(double solveTime)
{
    if (solveTime < 1.0)
        return solve(ob::timedPlannerTerminationCondition(solveTime));
    return solve(ob::timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)));
}

void ConstraintRespectingPlanner::printProperties(std::ostream &out) const
{
    out << "Planner " + getName() + " specs:" << std::endl;
    out << "Multithreaded:                 " << (getSpecs().multithreaded ? "Yes" : "No") << std::endl;
    out << "Reports approximate solutions: " << (getSpecs().approximateSolutions ? "Yes" : "No") << std::endl;
    out << "Can optimize solutions:        " << (getSpecs().optimizingPaths ? "Yes" : "No") << std::endl;
    out << "Aware of the following parameters:";
    std::vector<std::string> params;
    params_.getParamNames(params);
    for (auto &param : params)
        out << " " << param;
    out << std::endl;
}
  
void ConstraintRespectingPlanner::printSettings(std::ostream &out) const
{
    out << "Declared parameters for planner " << getName() << ":" << std::endl;
    params_.print(out);
}
  
void ob::PlannerInputStates::clear()
{
    if (tempState_ != nullptr)
    {
        si_->freeState(tempState_);
        tempState_ = nullptr;
    }
    addedStartStates_ = 0;
    sampledGoalsCount_ = 0;
    pdef_.reset();
    si_ = nullptr;
}
  
void ob::PlannerInputStates::restart()
{
    addedStartStates_ = 0;
    sampledGoalsCount_ = 0;
}
  
bool ob::PlannerInputStates::update()
{
    if (planner_ == nullptr)
        throw Exception("No planner set for PlannerInputStates");
    return use(planner_->getProblemDefinition());
}
  
void ob::PlannerInputStates::checkValidity() const
{
    std::string error;
 
    if (pdef_ == nullptr)
        error = "Problem definition not specified";
    else
    {
        if (pdef_->getStartStateCount() <= 0)
            error = "No start states specified";
        else if (!pdef_->getGoal())
            error = "No goal specified";
    }
  
    if (!error.empty())
    {
        if (planner_ != nullptr)
            throw Exception(planner_->getName(), error);
        else
            throw Exception(error);
    }
}
  
bool ob::PlannerInputStates::use(const ob::ProblemDefinitionPtr &pdef)
{
    if (pdef && pdef_ != pdef)
    {
        clear();
        pdef_ = pdef;
        si_ = pdef->getSpaceInformation().get();
        return true;
    }
    return false;
}
  
const ob::State *ob::PlannerInputStates::nextStart()
{
    if (pdef_ == nullptr || si_ == nullptr)
    {
        std::string error = "Missing space information or problem definition";
        if (planner_ != nullptr)
            throw Exception(planner_->getName(), error);
        else
            throw Exception(error);
    }
  
    while (addedStartStates_ < pdef_->getStartStateCount())
    {
        const base::State *st = pdef_->getStartState(addedStartStates_);
        addedStartStates_++;
        bool bounds = si_->satisfiesBounds(st);
        bool valid = bounds ? si_->isValid(st) : false;
        if (bounds && valid)
            return st;
  
        OMPL_WARN("%s: Skipping invalid start state (invalid %s)",
                  planner_ ? planner_->getName().c_str() : "PlannerInputStates", bounds ? "state" : "bounds");
        std::stringstream ss;
        si_->printState(st, ss);
        OMPL_DEBUG("%s: Discarded start state %s", planner_ ? planner_->getName().c_str() : "PlannerInputStates",
                   ss.str().c_str());
    }
    return nullptr;
}
  
const ob::State *ob::PlannerInputStates::nextGoal()
{
    // This initialization is safe since we are in a non-const function anyway.
    static ob::PlannerTerminationCondition ptc = plannerAlwaysTerminatingCondition();
    return nextGoal(ptc);
}
  
const ob::State *ob::PlannerInputStates::nextGoal(const ob::PlannerTerminationCondition &ptc)
{
    if (pdef_ == nullptr || si_ == nullptr)
    {
        std::string error = "Missing space information or problem definition";
        if (planner_ != nullptr)
            throw Exception(planner_->getName(), error);
        else
            throw Exception(error);
    }
  
    if (pdef_->getGoal() != nullptr)
    {
        const GoalSampleableRegion *goal =
            pdef_->getGoal()->hasType(GOAL_SAMPLEABLE_REGION) ? pdef_->getGoal()->as<GoalSampleableRegion>() : nullptr;
  
        if (goal != nullptr)
        {
            time::point start_wait;
            bool first = true;
            bool attempt = true;
            while (attempt)
            {
                attempt = false;
  
                if (sampledGoalsCount_ < goal->maxSampleCount() && goal->canSample())
                {
                    if (tempState_ == nullptr)
                        tempState_ = si_->allocState();
                    do
                    {
                        goal->sampleGoal(tempState_);
                        sampledGoalsCount_++;
                        bool bounds = si_->satisfiesBounds(tempState_);
                        bool valid = bounds ? si_->isValid(tempState_) : false;
                        if (bounds && valid)
                        {
                            if (!first)  // if we waited, show how long
                            {
                                OMPL_DEBUG("%s: Waited %lf seconds for the first goal sample.",
                                           planner_ ? planner_->getName().c_str() : "PlannerInputStates",
                                           time::seconds(time::now() - start_wait));
                            }
                            return tempState_;
                        }
  
                        OMPL_WARN("%s: Skipping invalid goal state (invalid %s)",
                                  planner_ ? planner_->getName().c_str() : "PlannerInputStates",
                                  bounds ? "state" : "bounds");
                        std::stringstream ss;
                        si_->printState(tempState_, ss);
                        OMPL_DEBUG("%s: Discarded goal state:\n%s",
                                   planner_ ? planner_->getName().c_str() : "PlannerInputStates", ss.str().c_str());
                    } while (!ptc && sampledGoalsCount_ < goal->maxSampleCount() && goal->canSample());
                }
                if (goal->couldSample() && !ptc)
                {
                    if (first)
                    {
                        first = false;
                        start_wait = time::now();
                        OMPL_DEBUG("%s: Waiting for goal region samples ...",
                                   planner_ ? planner_->getName().c_str() : "PlannerInputStates");
                    }
                    std::this_thread::sleep_for(time::seconds(0.01));
                    attempt = !ptc;
                }
            }
        }
    }
  
    return nullptr;
}
  
bool ob::PlannerInputStates::haveMoreStartStates() const
{
    if (pdef_ != nullptr)
        return addedStartStates_ < pdef_->getStartStateCount();
    return false;
}
  
bool ob::PlannerInputStates::haveMoreGoalStates() const
{
    if ((pdef_ != nullptr) && pdef_->getGoal())
        if (pdef_->getGoal()->hasType(GOAL_SAMPLEABLE_REGION))
            return sampledGoalsCount_ < pdef_->getGoal()->as<GoalSampleableRegion>()->maxSampleCount();
    return false;
}