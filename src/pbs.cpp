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
 
#include "includes/pbs.h"
// #include <limits>
// #include <ompl/tools/config/SelfConfig.h>
 
ompl::control::PBS::PBS(const std::vector<problem> mmpp): 
    base::Planner(mmpp[0].first, "PBS"), mmpp_{mmpp}
{
}

ompl::control::PBS::~PBS()
{
    // OMPL_ERROR("Desturctor called but not implemented");
}

ompl::base::PlannerStatus ompl::control::PBS::solve(const base::PlannerTerminationCondition &ptc)
{
    /* verify assumption that step size is the same for each planner */
    const auto *si = static_cast<const SpaceInformation *>(mmpp_[0].first.get());
    const double minStepSize = si->getPropagationStepSize();
    for (auto prob: mmpp_)
    {
       const auto *si = static_cast<const SpaceInformation *>(prob.first.get());
       const double tmpDT = si->getPropagationStepSize();
       if (tmpDT != minStepSize)
       {
          OMPL_ERROR("Planner Step Sizes are not the same for all agents!!");
          return base::PlannerStatus::INVALID_START;
       }
    }
    if (w_ == nullptr)
    {
        OMPL_ERROR("No world specified.");
        return base::PlannerStatus::INVALID_START;
    }

    /* create instances of low-level planner */
    std::vector<oc::timedRRT*> treeSearchs;
    for (int a = 0; a < mmpp_.size(); a++)
    {
        timedRRT* planner = new timedRRT(mmpp_[a].first);
        planner->setWorld(w_);
        planner->setProblemDefinition(mmpp_[a].second);
        planner->provideAgent(w_->getAgents()[a]);
        planner->setup();
        treeSearchs.push_back(planner);
    }
    /* plan */
    std::vector<PathControl> solution;
    OMPL_INFORM("%s: Starting planning. ", getName().c_str());
    auto start = std::chrono::high_resolution_clock::now();
    bool solved = false;
    for (auto p: treeSearchs)
    {
        p->setExistingSolutions(solution);
        ob::PlannerStatus solved = p->solve(ptc);
        /* store initial trajectory */
        if (solved)
        {
            PathControl traj = static_cast<PathControl &>
            (*p->ob::Planner::getProblemDefinition()->getSolutionPath());
            solution.push_back(traj);
        }
        else
            break;
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::microseconds>(stop - start);
    solveTime_ = (duration.count() / 1000000.0);
    if (solution.size() == treeSearchs.size())
        solved = true;
    /* clean up */
    for (auto p: treeSearchs)
    {
       p->clear();
       delete p;
    }
    if (solved)
    {
        OMPL_INFORM("PBS returned correct solution.");
        return {solved, false};
    }
    else
    {
        OMPL_INFORM("%s: No Solution Found.", getName().c_str());
        return {solved, false};
    }
}





