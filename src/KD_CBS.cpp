/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* The main source code of KD-CBS
*********************************************************************/
 
/* Author: Justin Kottinger */

#include "../includes/KD_CBS.h"


namespace base = ompl::base;

// constructor
ompl::control::KD_CBS::KD_CBS(const std::vector<SimpleSetup> &mmpp) : base::Planner(mmpp[0].getSpaceInformation(), "KD-CBS")
{
   mmpp_ = &mmpp;
   // These params are for benchmarking -- ignore for now
   // Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");
   // Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates,
                                // "0,1");
}

// de-constructor (TO-DO)
ompl::control::KD_CBS::~KD_CBS()
{
    // freeMemory();
}

// the main algorithm
base::PlannerStatus ompl::control::KD_CBS::solve(const base::PlannerTerminationCondition &ptc)
{ 
   // create initial solution
   Plan root_sol;
   for (SimpleSetup ss: *mmpp_)
   {
      base::PlannerStatus solved = ss.solve(planningTime_);
      ss.getPlanner()->clear();
      if (solved)
         root_sol.push_back(ss.getSolutionPath());
   }
   // create root node
   conflictNode *rootNode = new conflictNode();
   if (root_sol.size() == mmpp_->size())
   {
      rootNode->updatePlanAndCost(root_sol);
      queue_.insert(rootNode);
   }
 
   // if no root node exists, tell user and return invalid start
   if (queue_.size() == 0)
   {
       OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
       return base::PlannerStatus::INVALID_START;
   }
   
   // begin planning -- timing should start after this statement
   OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), queue_.size());
 
   // initialize solution, approximate solution, and approximate difference between approx and sol.
   conflictNode *solution = nullptr;
   conflictNode *approxsol = nullptr;
   double approxdif = std::numeric_limits<double>::infinity();
  
   while (ptc == false && !queue_.empty())
   { 
      // /* find loswest cost in Queue */
      // conflictNode *current = popHead();
      OMPL_WARN("In the main loop");

      conflictNode *curr = popHead();

      conflictNode *another = new conflictNode();

      // dummy constraints (TODO)
      std::vector<int> c{1, 2, 3, 4, 5};
      
      // cast to oc::SimpleSetUp for the second agent
      // SimpleSetup ss = (*mmpp_)[1];
      // includeConstraints(c, 1);
      // base::PlannerStatus solved = ss.solve(planningTime_);
      // if (solved)
      //    new_sol.push_back(ss.getSolutionPath());
      // ss.getPlanner()->clear();
      int aIdx = 1;
      bool updated = false;
      PathControl new_traj = replanSingleAgent(c, aIdx, curr->getPlan()[aIdx], updated);

      if (updated)
         OMPL_WARN("After Replan Function, trajectories were UPDATED!");

      exit(1);

      // queue_.insert(another);

      // while (!.queue_.empty())
      // {
      //    conflictNode
      // }

 
      // /* sample a random control that attempts to go towards the random state, and also sample a control duration */
      // unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
 
      // if (addIntermediateStates_)
      // {
      //    // this code is contributed by Jennifer Barry
      //    std::vector<base::State *> pstates;
      //    cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);
 
      //    if (cd >= siC_->getMinControlDuration())
      //    {
      //       Motion *lastmotion = nmotion;
      //       bool solved = false;
      //       size_t p = 0;
      //       for (; p < pstates.size(); ++p)
      //       {
      //           /* create a motion */
      //           auto *motion = new Motion();
      //           motion->state = pstates[p];
      //           // we need multiple copies of rctrl
      //           motion->control = siC_->allocControl();
      //           siC_->copyControl(motion->control, rctrl);
      //           motion->steps = 1;
      //           motion->parent = lastmotion;
      //           lastmotion = motion;
      //           nn_->add(motion);
      //           double dist = 0.0;
      //           solved = goal->isSatisfied(motion->state, &dist);
      //           if (solved)
      //           {
      //               approxdif = dist;
      //               solution = motion;
      //               break;
      //           }
      //           if (dist < approxdif)
      //           {
      //               approxdif = dist;
      //               approxsol = motion;
      //           }
      //       }
 
      //       // free any states after we hit the goal
      //       while (++p < pstates.size())
      //           si_->freeState(pstates[p]);
      //       if (solved)
      //           break;
      //       }
      //       else
      //          for (auto &pstate : pstates)
      //             si_->freeState(pstate);
      // }
      // else
      // {
      //    if (cd >= siC_->getMinControlDuration())
      //    {
      //       /* create a motion */
      //       auto *motion = new Motion(siC_);
      //       si_->copyState(motion->state, rmotion->state);
      //       siC_->copyControl(motion->control, rctrl);
      //       motion->steps = cd;
      //       motion->parent = nmotion;
 
      //       nn_->add(motion);
      //       double dist = 0.0;
      //       bool solv = goal->isSatisfied(motion->state, &dist);
      //       if (solv)
      //       {
      //           approxdif = dist;
      //           solution = motion;
      //           break;
      //       }
      //       if (dist < approxdif)
      //       {
      //           approxdif = dist;
      //           approxsol = motion;
      //       }
      //    }
      // }
   }
   OMPL_WARN("Made it here. ");
   // end main algorithm -- begin ompl bookkeeping 
   bool solved = false;
   bool approximate = false;
   if (solution == nullptr)
   {
      solution = approxsol;
      approximate = true;
   }

   // // solution is either approxsol or solution from main loop
   // if (solution != nullptr)
   // {
   //    // set last node to solution
   //    lastGoalNode_ = solution;
 
   //    /* construct the solution path */
   //    std::vector<Motion *> mpath;
   //    while (solution != nullptr)
   //    {
   //       mpath.push_back(solution);
   //       solution = solution->parent;
   //    }
 
   //    /* set the solution path */
   //    auto path(std::make_shared<PathControl>(si_));
   //    for (int i = mpath.size() - 1; i >= 0; --i)
   //       if (mpath[i]->parent)
   //          path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
   //       else
   //          path->append(mpath[i]->state);
   //    solved = true;
   //    pdef_->addSolutionPath(path, approximate, approxdif, getName());
   // }
 
   // if (rmotion->state)
   //    si_->freeState(rmotion->state);
   // if (rmotion->control)
   //    siC_->freeControl(rmotion->control);
   // delete rmotion;
   // si_->freeState(xstate);
 
   OMPL_INFORM("%s: Created %u states", getName().c_str(), queue_.size());
 
   return {solved, approximate};
}
