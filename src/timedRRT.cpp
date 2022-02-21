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
 
/* Author: Justin Kottinger */
 
#include "includes/timedRRT.h"

namespace ob = ompl::base;

 
ompl::control::timedRRT::timedRRT(const SpaceInformationPtr &si) : base::Planner(si, "Timed-RRT")
{
    specs_.approximateSolutions = false;
    siC_ = si.get();
 
    Planner::declareParam<double>("goal_bias", this, &timedRRT::setGoalBias, &timedRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &timedRRT::setIntermediateStates, &timedRRT::getIntermediateStates,
                                "0,1");
}
 
ompl::control::timedRRT::~timedRRT()
{
    freeMemory();
}
 
void ompl::control::timedRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}
 
void ompl::control::timedRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}
 
void ompl::control::timedRRT::freeMemory()
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

bool ompl::control::timedRRT::isCollisionFree(Motion* current)
{
    if (existingSolns_.empty())
        return true;

    /* construct the path segment */
    Motion *m = current;
    std::vector<Motion *> mpath;
    while (m != nullptr)
    {
        mpath.push_back(m);
        m = m->parent;
    }
 
    /* set the path segment */
    PathControl pathSegment(si_);
    for (int i = mpath.size() - 1; i >= 0; --i)
        if (mpath[i]->parent)
            pathSegment.append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
        else
            pathSegment.append(mpath[i]->state);

    // interpolate the path to match existing solutions
    pathSegment.interpolate();

    int currStateStep = pathSegment.getStateCount()-1;

    // for every state in pathSegment, check disjointedness against existingSolutions
    // for (int k = 0; k < pathSegment.getStateCount(); k++)
    // {
    // create agent
    const double agent_width = a_->getShape()[0];
    const double agent_height = a_->getShape()[1];
    // extract points from state object
    auto compState = current->state->as<ob::CompoundStateSpace::StateType>();
    auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
    const double agent_cx = xyState->values[0];
    const double agent_cy = xyState->values[1];
    const double theta = compState->as<
        ob::SO2StateSpace::StateType>(1)->value;
    // const double theta = 0.0;
    // turn (x,y, theta), width, length to a polygon object
    // borrowed from conflictChecking
    // TOP RIGHT VERTEX:
    const double TR_x = agent_cx + ((agent_width / 2) * cos(theta)) - ((agent_height / 2) * sin(theta));
    const double TR_y = agent_cy + ((agent_width / 2) * sin(theta)) + ((agent_height / 2) * cos(theta));
    std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
    // TOP LEFT VERTEX:
    const double TL_x = agent_cx - ((agent_width / 2) * cos(theta)) - ((agent_height / 2) * sin(theta));
    const double TL_y = agent_cy - ((agent_width / 2) * sin(theta)) + ((agent_height / 2) * cos(theta));
    std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
    // BOTTOM LEFT VERTEX:
    const double BL_x = agent_cx - ((agent_width / 2) * cos(theta)) + ((agent_height / 2) * sin(theta));
    const double BL_y = agent_cy - ((agent_width / 2) * sin(theta)) - ((agent_height / 2) * cos(theta));
    std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
    // BOTTOM RIGHT VERTEX:
    const double BR_x = agent_cx + ((agent_width / 2) * cos(theta)) + ((agent_height / 2) * sin(theta));
    const double BR_y = agent_cy + ((agent_width / 2) * sin(theta)) - ((agent_height / 2) * cos(theta));
    std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);
    // convert to string for easy initializataion
    std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
    polygon planning_agent;
    boost::geometry::read_wkt(points, planning_agent);
        
    for (int agentIdx = 0; agentIdx < existingSolns_.size(); agentIdx++)
    {
        PathControl existingPath = existingSolns_[agentIdx];
        // // does exisiting path at k exist?
        // bool check = false;
        if (currStateStep < existingPath.getStateCount())
        {
        // for (int k = 0; k < existingPath.getStateCount(); k++)
        // {
        //     if (currStateStep == k)
        //     {
                // check = true;
                // create other agent shape
                const Agent *otherAgent = w_->getAgents()[agentIdx];
                const double otherAgent_width = otherAgent->getShape()[0];
                const double otherAgent_height = otherAgent->getShape()[1];
                // extract points from state object
                auto compState = existingPath.getState(currStateStep)->as<ob::CompoundStateSpace::StateType>();
                auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
                const double otherAgent_cx = xyState->values[0];
                const double otherAgent_cy = xyState->values[1];
                const double theta = compState->as<
                    ob::SO2StateSpace::StateType>(1)->value;
                // const double theta = 0.0;
                // turn (x,y, theta), width, length to a polygon object
                // borrowed from conflictChecking
                // TOP RIGHT VERTEX:
                const double TR_x = otherAgent_cx + ((otherAgent_width / 2) * cos(theta)) - ((otherAgent_height / 2) * sin(theta));
                const double TR_y = otherAgent_cy + ((otherAgent_width / 2) * sin(theta)) + ((otherAgent_height / 2) * cos(theta));
                std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
                // TOP LEFT VERTEX:
                const double TL_x = otherAgent_cx - ((otherAgent_width / 2) * cos(theta)) - ((otherAgent_height / 2) * sin(theta));
                const double TL_y = otherAgent_cy - ((otherAgent_width / 2) * sin(theta)) + ((otherAgent_height / 2) * cos(theta));
                std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
                // BOTTOM LEFT VERTEX:
                const double BL_x = otherAgent_cx - ((otherAgent_width / 2) * cos(theta)) + ((otherAgent_height / 2) * sin(theta));
                const double BL_y = otherAgent_cy - ((otherAgent_width / 2) * sin(theta)) - ((otherAgent_height / 2) * cos(theta));
                std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
                // BOTTOM RIGHT VERTEX:
                const double BR_x = otherAgent_cx + ((otherAgent_width / 2) * cos(theta)) + ((otherAgent_height / 2) * sin(theta));
                const double BR_y = otherAgent_cy + ((otherAgent_width / 2) * sin(theta)) - ((otherAgent_height / 2) * cos(theta));
                std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);
                // convert to string for easy initializataion
                std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
                polygon other_agent;
                boost::geometry::read_wkt(points,other_agent);

                // check if planning_agent is disjoint from other_agent
                if (! boost::geometry::disjoint(planning_agent, other_agent))
                {
                    // if (abs(TR_x_a - TR_x) <= 0.2)
                    // {
                    //     std::cout << "Existing: " << boost::geometry::wkt(other_agent) << std::endl;
                    //     std::cout << "Planning: " << boost::geometry::wkt(planning_agent) << std::endl;
                    //     std::cout << "Returned False" << std::endl;
                    // }
                    return false;
                }
                // else
                // {
                //     if (abs(agent_cx - otherAgent_cx) <= 0.5)
                //     {
                //         std::cout << "Existing: " << boost::geometry::wkt(other_agent) << std::endl;
                //         std::cout << "Planning: " << boost::geometry::wkt(planning_agent) << std::endl;
                //         std::cout << "Returned True" << std::endl;
                //         exit(1);
                //     }
                // }
            // }
        }
        else
        {
            // need to check final state
            // create other agent shape
            const Agent *otherAgent = w_->getAgents()[agentIdx];
            const double otherAgent_width = otherAgent->getShape()[0];
            const double otherAgent_height = otherAgent->getShape()[1];
            // extract points from state object
            auto compState = existingPath.getStates().back()->as<ob::CompoundStateSpace::StateType>();
            auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
            const double otherAgent_cx = xyState->values[0];
            const double otherAgent_cy = xyState->values[1];
            const double theta = compState->as<
                ob::SO2StateSpace::StateType>(1)->value;
            // const double theta = 0.0;
            // turn (x,y, theta), width, length to a polygon object
            // borrowed from conflictChecking
            // TOP RIGHT VERTEX:
            const double TR_x = otherAgent_cx + ((otherAgent_width / 2) * cos(theta)) - ((otherAgent_height / 2) * sin(theta));
            const double TR_y = otherAgent_cy + ((otherAgent_width / 2) * sin(theta)) + ((otherAgent_height / 2) * cos(theta));
            std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
            // TOP LEFT VERTEX:
            const double TL_x = otherAgent_cx - ((otherAgent_width / 2) * cos(theta)) - ((otherAgent_height / 2) * sin(theta));
            const double TL_y = otherAgent_cy - ((otherAgent_width / 2) * sin(theta)) + ((otherAgent_height / 2) * cos(theta));
            std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
            // BOTTOM LEFT VERTEX:
            const double BL_x = otherAgent_cx - ((otherAgent_width / 2) * cos(theta)) + ((otherAgent_height / 2) * sin(theta));
            const double BL_y = otherAgent_cy - ((otherAgent_width / 2) * sin(theta)) - ((otherAgent_height / 2) * cos(theta));
            std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
            // BOTTOM RIGHT VERTEX:
            const double BR_x = otherAgent_cx + ((otherAgent_width / 2) * cos(theta)) + ((otherAgent_height / 2) * sin(theta));
            const double BR_y = otherAgent_cy + ((otherAgent_width / 2) * sin(theta)) - ((otherAgent_height / 2) * cos(theta));
            std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);
            // convert to string for easy initializataion
            std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
            polygon other_agent;
            boost::geometry::read_wkt(points,other_agent);

            // check if planning_agent is disjoint from other_agent
            if (! boost::geometry::disjoint(planning_agent, other_agent))
            {
                // if (abs(TR_x_a - TR_x) <= 0.2)
                // {
                //     std::cout << "Existing: " << boost::geometry::wkt(other_agent) << std::endl;
                //     std::cout << "Planning: " << boost::geometry::wkt(planning_agent) << std::endl;
                //     std::cout << "Returned False" << std::endl;
                // }
                return false;
            }
        }
    }
    return true;
}
 
ompl::base::PlannerStatus ompl::control::timedRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    // check validity of solver given additional constraints
    if (w_ == nullptr)
    {
        OMPL_ERROR("World Not Specified!");
        return base::PlannerStatus::INVALID_START;
    }
    if (!existingSolns_.empty())
    {
        // interpolate all paths prior to planning
        for (int p = 0; p < existingSolns_.size(); p++)
            existingSolns_[p].interpolate();
    }
    // begin normal planning
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
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
 
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

                if (isCollisionFree(motion))
                {
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
                else
                {
                    delete motion;
                }
            }
        }
    }
 
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
 
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
 
    return {solved, approximate};
}
 
void ompl::control::timedRRT::getPlannerData(base::PlannerData &data) const
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