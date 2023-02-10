/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
 
/* Author: Ioan Sucan */
 
#include "Planners/MultiRobotRRT.h"


ompl::control::MultiRobotRRT::MultiRobotRRT(const SpaceInformationPtr &si) : base::Planner(si, "MA-RRT")
{
    specs_.approximateSolutions = false;
    siC_ = si.get();
 
    Planner::declareParam<double>("goal_bias", this, &MultiRobotRRT::setGoalBias, &MultiRobotRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &MultiRobotRRT::setIntermediateStates, &MultiRobotRRT::getIntermediateStates,
                                "0,1");
}
 
ompl::control::MultiRobotRRT::~MultiRobotRRT()
{
    freeMemory();
}
 
void ompl::control::MultiRobotRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}
 
void ompl::control::MultiRobotRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}
 
void ompl::control::MultiRobotRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

unsigned int ompl::control::MultiRobotRRT::MultiAgentControlSampler(Control *rcontrol, Control *previous, 
    const base::State *source, base::State *dest)
{
    // determine which agents are already in goal
    auto g = getProblemDefinition()->getGoal()->as<R2MultiRobotGoal>();
    std::vector<int> idxInGoal = g->isInGoal(source);

    // propogate as normal
    /* sample a random control that attempts to go towards the random state, and also sample a control duration */
    unsigned int cd = controlSampler_->sampleTo(rcontrol, previous, source, dest);
    /* override destination for agents already in goal */
    overrideStates(idxInGoal, source, dest, rcontrol); 
    return cd;
}

void ompl::control::MultiRobotRRT::overrideStates(const std::vector<int> DoNotProp, const base::State *source, 
    base::State *result, Control *control)
{
    // this function takes in a state and a list of vehicle indexes that do not need to be propogated
    // it changes the state s.t. state propogation and controls are null for any vehicles in the goal
    if (DoNotProp.size() > 0)
    {
        // this array has a list of vehicles that do not need to be progpogated
        // we need to iterate through all of them

        // get the entire compound state for result
        auto destination = result->as<ompl::base::CompoundStateSpace::StateType>();

        // get the entire compound state space for source
        const auto *src = source->as<ob::CompoundStateSpace::StateType>();

        auto *cntrl = control->as<RealVectorControlSpace::ControlType>()->values;

        for (int i = 0; i < DoNotProp.size(); i++)
        {
            // overriding controls 
            // indexing method: ControlDimension(vehicle index) + ControlIndex
            cntrl[2*DoNotProp[i] + 0] = 0.0;
            cntrl[2*DoNotProp[i] + 1] = 0.0;

            // next, make the source state the destination state for the indiv. vehicles
            // indexing method: 2(vehicle) or 2(vehicle) + 1

            // get the specific xy state of result 
            auto *xyDest = destination->as<base::RealVectorStateSpace::StateType>(2*DoNotProp[i]);

            // get the specific xy state of the source
            const auto *xySrc = src->as<base::RealVectorStateSpace::StateType>(2*DoNotProp[i]);

            // get the specific orientation of result
            auto *rotDest = destination->as<base::SO2StateSpace::StateType>(2*DoNotProp[i] + 1);

            // get the specific orientation of the source
            const auto *rotSrc = src->as<base::SO2StateSpace::StateType>(2*DoNotProp[i] + 1);

            // overriding the position state
            xyDest->values[0] = xySrc->values[0];
            xyDest->values[1] = xySrc->values[1];

            // overriding the orientation
            rotDest->value = rotSrc->value;

        }
    }
}
 
ompl::base::PlannerStatus ompl::control::MultiRobotRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
 
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
    }
 
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
 
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();
 
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
    auto start = std::chrono::high_resolution_clock::now();
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
 
    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();
 
    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);
 
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
 
        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        // unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
        unsigned int cd = MultiAgentControlSampler(rctrl, nmotion->control, nmotion->state, rmotion->state);
 
        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);
 
            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for (; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    auto *motion = new Motion();
                    motion->state = pstates[p];
                    // we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    nn_->add(motion);
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }
 
                // free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (auto &pstate : pstates)
                    si_->freeState(pstate);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* create a motion */
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;
 
                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
            }
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    bool solved = false;
    bool approximate = false;
    // if (solution == nullptr)
    // {
    //     solution = approxsol;
    //     approximate = true;
    // }
 
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;
 
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }
 
        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }
 
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    solveTime_ = (duration.count() / 1000000.0);
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    clear();
    return {solved, approximate};
}
 
void ompl::control::MultiRobotRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
 
    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);
 
    double delta = siC_->getPropagationStepSize();
 
    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
 
    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}