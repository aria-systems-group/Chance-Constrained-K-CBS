/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* The main source code of KD-CBS
*********************************************************************/
 
/* Author: Justin Kottinger */

#include "Planners/KCBS.h"


// constructor
ompl::control::KCBS::KCBS(const MultiRobotProblemDefinitionPtr mrmp_pdef) : 
   base::Planner(mrmp_pdef->getRobotSpaceInformationPtr(0), "K-CBS"), 
   mrmp_pdef_(mrmp_pdef), ready_(false)
{
   setUp_();
   // These params are for benchmarking -- ignore for now
   // Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");
   // Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates,
                                // "0,1");
   if (ready_)
      OMPL_INFORM("%s: Ready to Plan.", getName().c_str());
   else {
      OMPL_ERROR("%s: Set-Up Proccedures failed.", getName().c_str());
   }
}

void ompl::control::KCBS::setUp_()
{
   const std::vector<MotionPlanningProblemPtr> mrmp_info = mrmp_pdef_->getAllProblemInformation();

   /* verify assumption that step size is the same for each planner */
   const double sys_dt = mrmp_pdef_->getSystemStepSize();
   for (auto itr = mrmp_info.begin() + 1; itr != mrmp_info.end(); itr++) {
      const oc::SpaceInformationPtr siPtr = (*itr)->getSpaceInformation();
      const double dt = siPtr->getPropagationStepSize();
      if (dt != sys_dt) {
         return;
      }
   }

   /* create (and include) instances of low-level planner */
   for (auto itr = mrmp_info.begin(); itr != mrmp_info.end(); itr++) {
      if ( (*itr)->getPlanner() ) {
         low_level_planners_.push_back((*itr)->getPlanner());
      }
   }

   if (low_level_planners_.size() != mrmp_info.size()) {
      return;
   }

   /* initialize disjoint sets */
   for (int a1 = 0; a1 < mrmp_info.size(); a1++)
   {
      for (int a2 = 0; a2 < mrmp_info.size(); a2++)
      {
         if (a1 != a2)
         {
            // does (a1, a2) exist in conf_counter_?
            bool add = true;
            for (int i = 0; i < conf_counter_.size(); i++)
            {
               std::pair<int, int> curr_pair = conf_counter_[i].first;
               if (curr_pair.first == a1 && curr_pair.second == a2)
                  add = false;
               else if (curr_pair.first == a2 && curr_pair.second == a1)
                  add = false;
               if (add == false)
                  break;
            }
            if (add)
            {
               std::pair<int, int> agnt_pair{a1, a2};
               std::pair< std::pair<int, int>, int > conflict_count{agnt_pair, 0};
               conf_counter_.push_back(conflict_count);
            }
         }
      }
   }

   ready_ = true;
}

// de-constructor (TO-DO)
ompl::control::KCBS::~KCBS()
{
    freeMemory_();
}

// free all alocated memory
void ompl::control::KCBS::freeMemory_()
{
   OMPL_WARN("%s: freeMemory_ called but not yet implemented. Possible memory leak.", getName().c_str());
}

// Plan ompl::control::KCBS::lowLevelSearch(oc::ConstraintRRT* p, std::vector<const Constraint*> &constraints, 
   // const Plan parentPlan, bool replanning)
// {
//    Plan tmp_plan;
//    if (!bypass_ && !replanning)
//    {
//       /* no bypassing. proceed as usual */
//       p->updateConstraints(constraints);
//       ob::PlannerStatus solved = p->ob::Planner::solve(planningTime_);
//       if (solved)
//       {
//          OMPL_INFORM("Seccessfully Replanned.");
//          /* create new solution with updated traj. for conflicting agent */
//          oc::PathControl new_traj = static_cast<oc::PathControl &>
//             (*p->ob::Planner::getProblemDefinition()->getSolutionPath());
//          for (int agent = 0; agent < mmpp_.size(); agent++)
//          {
//             if (constraints.front()->getAgent() == agent)
//                tmp_plan.push_back(new_traj);
//             else
//                tmp_plan.push_back(parentPlan[agent]);
//          }
//          return tmp_plan;
//       }
//       else
//       {
//          OMPL_INFORM("Failed to replan.");
//          return tmp_plan; // empty
//       }
//    }
//    else if (replanning)  // no bypassing performed when replanning
//    {
//       // plan again
//       ob::PlannerStatus solved = p->ob::Planner::solve(planningTime_);
//       if (solved)
//       {
//          /* create new solution with updated traj. for conflicting agent */
//          oc::PathControl new_traj = static_cast<oc::PathControl &>
//             (*p->ob::Planner::getProblemDefinition()->getSolutionPath());
//          for (int agent = 0; agent < mmpp_.size(); agent++)
//          {
//             if (constraints.front()->getAgent() == agent)
//                tmp_plan.push_back(new_traj);
//             else
//                tmp_plan.push_back(parentPlan[agent]);
//          }
//          return tmp_plan;
//       }
//       else
//       {
//          OMPL_INFORM("Failed to replan.");
//          return tmp_plan; // empty
//       }
//    }
//    else if (bypass_ && !replanning)
//    {
//       auto start = std::chrono::high_resolution_clock::now();
//       auto stop = std::chrono::high_resolution_clock::now();
//       auto duration = duration_cast<std::chrono::microseconds>(stop - start);
//       double timeLeft = planningTime_ - (duration.count() / 1000000.0);
//       int bp = 0;
//       while (timeLeft > 0)
//       {
//          if (bp == 0)
//             p->updateConstraints(constraints);
//          else
//             p->pruneTree(constraints);
//          ob::PlannerStatus solved = p->ob::Planner::solve(timeLeft);
//          if (solved)
//          {
//             OMPL_INFORM("Seccessfully Replanned. Attempting to Bypass.");
//             /* create new solution with updated traj. for conflicting agent */
//             oc::PathControl new_traj = static_cast<oc::PathControl &>
//                (*p->ob::Planner::getProblemDefinition()->getSolutionPath());
//             OMPL_INFORM("Got Traj.");
//             if (bp == 0)
//             {
//                for (int agent = 0; agent < mmpp_.size(); agent++)
//                {
//                   if (constraints.front()->getAgent() == agent)
//                      tmp_plan.push_back(new_traj);
//                   else
//                      tmp_plan.push_back(parentPlan[agent]);
//                }
//             }
//             else
//             {
//                // replace rather that append to tmp_plan
//                tmp_plan.erase(tmp_plan.begin() +  constraints.front()->getAgent());
//                tmp_plan.insert(tmp_plan.begin() +  constraints.front()->getAgent(), new_traj);

//             }
//             // validate tmp_plan
//             std::vector <Conflict> conf = validatePlan(tmp_plan);
//             if (conf.empty())
//             {
//                OMPL_INFORM("Collision Free Plan found.");
//                return tmp_plan;
//             }
//             else
//             {
//                OMPL_INFORM("Conflict found.");
//                /* extract conflict information*/
//                const std::vector<int> conflicting_agents
//                   {conf.front().agent1, conf.front().agent2};
//                std::vector<std::vector<polygon>> conflicting_polys{ {}, {} };
//                std::vector<double> conflicting_times{};
//                /* fill time and polygon vectors*/
//                for (Conflict c: conf)
//                {
//                   conflicting_times.push_back(c.time);
//                   conflicting_polys[0].push_back(c.p2);
//                   conflicting_polys[1].push_back(c.p1);
//                }
//                OMPL_INFORM("Conflict between agents: (%d, %d)", conf.front().agent1, conf.front().agent2);
//                OMPL_INFORM("Conflict Time Range: [%0.1f, %0.1f]", conflicting_times.front(), conflicting_times.back());
//                // constraints.front().getAgent must be in conf to bypass
//                if (conf.front().agent1 == constraints.front()->getAgent())
//                {
//                   /* init new constraint and trajectory to resolve conflict */
//                   auto *new_constraint = new Constraint(conflicting_polys[0], 
//                      conflicting_times, conflicting_agents[0]);
//                   constraints.push_back(new_constraint);
//                }
//                else if (conf.front().agent2 == constraints.front()->getAgent())
//                {
//                   /* init new constraint and trajectory to resolve conflict */
//                   auto *new_constraint = new Constraint(conflicting_polys[1], 
//                      conflicting_times, conflicting_agents[1]);
//                   constraints.push_back(new_constraint);
//                }
//                else
//                {
//                   OMPL_INFORM("Unable to bypass.");
//                   return tmp_plan;
//                }
//             }
//          }
//          else
//          {
//             OMPL_INFORM("Failed to replan.");
//             return tmp_plan; // empty
//          }
//          bp++;
//          stop = std::chrono::high_resolution_clock::now();
//          duration = duration_cast<std::chrono::microseconds>(stop - start);
//          timeLeft = planningTime_ - (duration.count() / 1000000.0);
//       }
//       return tmp_plan;
//    }
//    else
//    {
//       OMPL_ERROR("Bypassing not implemented properly.");
//       exit(1);
//       return tmp_plan;
//    }
// }

oc::PathControl* ompl::control::KCBS::calcNewPath_(ConstraintRespectingPlannerPtr planner, std::vector<ConstraintPtr> constraints)
{
   oc::PathControl *traj = nullptr;
   planner->updateConstraints(constraints);
   ob::PlannerStatus solved = planner->solve(planningTime_);
   if (solved)
   {
      OMPL_INFORM("%s: Successfully Replanned.", getName().c_str());
      /* create new solution with updated traj. for conflicting agent */
      traj = planner->getProblemDefinition()->getSolutionPath()->as<oc::PathControl>();
      return traj;
   }
   else
   {
      OMPL_INFORM("Failed to replan.");
      return traj; // empty
   }
}


bool ompl::control::KCBS::shouldMerge_(
   std::vector< std::pair< std::pair<int, int>, int> > &conf_cntr, 
   const int agent1, const int agent2)
{
   // printf("(%i, %i)\n", agent1, agent2);
   // first, update the conflict tracking
   for (int i = 0; i < conf_cntr.size(); i++)
   {
      // printf("Checking pair: (%i, %i) \n", conf_cntr[i].first.first, conf_cntr[i].first.second);
      if ((conf_cntr[i].first.first == agent1) && (conf_cntr[i].first.second == agent2))
      {
         // printf("I am here\n");
         conf_cntr[i].second = conf_cntr[i].second + 1;
         // printf("Changed value: %i \n", conf_cntr[i].second);
         break;
      }
      else if ((conf_cntr[i].first.first == agent2) && (conf_cntr[i].first.second == agent1))
      {
         // printf("I am down here.\n");
         conf_cntr[i].second = conf_cntr[i].second + 1;
         break;
      }
   }

   for (int i = 0; i < conf_cntr.size(); i++)
   {
      printf("Pair: (%i, %i) Num. Conflicts: %i \n", conf_cntr[i].first.first, conf_cntr[i].first.second, conf_cntr[i].second);
   }

   // second, figure out if we should merge
   for (int i = 0; i < conf_cntr.size(); i++)
   {
      if (conf_cntr[i].second >= B_)
         return true;
   }
   return false;
}

// Instance* ompl::control::KCBS::composeSystem(const int agentIdx1, const int agentIdx2)
// {
//    std::pair<int, int> idxs{agentIdx1, agentIdx2};
//    merger_count_.push_back(idxs);
//    std::vector<problem_t> x;
//    /* first, change the world params to match new problem.
//       world object has a vector of Agents. need to replace the two 
//       at agent1 and agent2 idx and merge them into a single agent*/

//    mrmp_instance_->printRobots();

//    // create new world
//    Instance *new_instance = new Instance(*mrmp_instance_);
//    // // give map dimensions
//    // new_instance->setDimensions(mrmp_instance_->getDimensions()[0], mrmp_instance_->getDimensions()[1]);
//    // // provide obstacles
//    // for (auto o: mrmp_instance_->getObstacles())
//    //    new_instance->addObstacle(o);
//    // // provide agents (except the ones that will be merged)
//    // for(int a = 0; a < mrmp_instance_->getAgents().size(); a++)
//    // {
//    //    if ((a != agentIdx1) && (a!= agentIdx2))
//    //       new_instance->addAgent(mrmp_instance_->getRobots()[a]);
//    // }

//    // /* add new agent */
//    // if ((mrmp_instance_->getRobots()[agentIdx1]->getDynamicsModel() == "Dynamic Car") && 
//    //     (mrmp_instance_->getRobots()[agentIdx2]->getDynamicsModel() == "Dynamic Car"))
//    // {
//    //    OMPL_INFORM("%s: Creating a %s model", getName().c_str(), "Two Dynamic Cars");

//    //    // std::vector<double> s, std::vector<double> g
//    //    std::string new_name = "agent" + std::to_string(new_world->getAgents().size());
//    //    std::vector<double> new_start{
//    //       mrmp_instance_->getRobots()[agentIdx1]->getStartLocation()[0], 
//    //       mrmp_instance_->getRobots()[agentIdx1]->getStartLocation()[1],
//    //       mrmp_instance_->getRobots()[agentIdx2]->getStartLocation()[0],
//    //       mrmp_instance_->getRobots()[agentIdx2]->getStartLocation()[1]};
//    //    std::vector<double> new_goal{
//    //       mrmp_instance_->getRobots()[agentIdx1]->getGoalLocation()[0], 
//    //       mrmp_instance_->getRobots()[agentIdx1]->getGoalLocation()[1],
//    //       mrmp_instance_->getRobots()[agentIdx2]->getGoalLocation()[0],
//    //       mrmp_instance_->getRobots()[agentIdx2]->getGoalLocation()[1]};

//    //    Agent* new_agent = new Agent(new_name, "Two Dynamic Cars", 
//    //       mrmp_instance_->getRobots()[0]->getShape(), new_start, new_goal);

//    //    new_instance->addRobot(new_agent);

//    //    new_instance->printRobots();
//    // }
//    // else
//    // {
//    //    OMPL_ERROR("%s: unable to compose %s and %s. Implementation must be extended.",
//    //       getName().c_str(), mrmp_instance_->getRobots()[agentIdx1]->getDynamicsModel().c_str(), 
//    //       mrmp_instance_->getRobots()[agentIdx2]->getDynamicsModel().c_str());
//    //    return nullptr;
//    // }
//    OMPL_ERROR("This portion of code has not yet been updated after refactor!");
//    return new_instance;
// }

// the main algorithm
ob::PlannerStatus ompl::control::KCBS::solve(const base::PlannerTerminationCondition &ptc)
{ 
   if (!ready_) {
      OMPL_ERROR("%s: Invalid Set-up. Unable to plan.", getName().c_str());
      return base::PlannerStatus::INVALID_START;
   }

   /* initialize priority queue and constants */ 
   std::priority_queue<KCBSNode, std::vector<KCBSNode>, Compare> pq;
   std::vector<MotionPlanningProblemPtr> all_mp_pdefs = mrmp_pdef_->getAllProblemInformation();

   /* begin planning -- timing should start after this statement */
   OMPL_INFORM("%s: Starting planning. ", getName().c_str());
   auto start = std::chrono::high_resolution_clock::now();

   // /* create initial solution */
   Plan root_plan;
   for (auto itr = low_level_planners_.begin(); itr != low_level_planners_.end(); itr++)
   {
      ob::PlannerStatus solved = (*itr)->solve(planningTime_);
      while (!solved && !ptc){
         solved = (*itr)->solve(planningTime_);
         if (solved == base::PlannerStatus::INVALID_START) {
            return base::PlannerStatus::INVALID_START;
         }
      }

      /* store initial trajectory */
      if (solved)
      {
         oc::PathControl* traj = (*itr)->getProblemDefinition()->getSolutionPath()->as<oc::PathControl>();
         root_plan.push_back(*traj);
      }
      (*itr)->clear();
   }

   /* create root node */
   KCBSNode rootNode;
   if (root_plan.size() == all_mp_pdefs.size())
   {
      rootNode.updatePlanAndCost(root_plan);
      pq.emplace(rootNode);
   }
 
   /* initialize solution */
   KCBSNode *solution = nullptr;
   while (ptc == false && !pq.empty())
   {
      /* get the lowest cost in priority queue */
      auto curr = new KCBSNode(pq.top());

      /* if cost==inf need to try to re-plan node */
      if (curr->getCost() == std::numeric_limits<double>::infinity())
      {
         pq.pop();
         OMPL_WARN("%s: In Merge block but not implemented!", getName().c_str());
         return {false, false};
         // // printf("Now Here: %lu \n", curr->getMotions().size());
         // // update planner information
         // const int agentIdx = curr->getConstraint()->getAgent();
         // // get constraints
         // std::vector<const Constraint*> agent_constraints{curr->getConstraint()};
         // const conflictNode *nCpy = curr->getParent();
         // while (nCpy->getConstraint() != nullptr)
         // {
         //    if (nCpy->getConstraint()->getAgent() == agentIdx)
         //       agent_constraints.push_back(nCpy->getConstraint());
         //    nCpy = nCpy->getParent();
         // }
//          // add constraints and motions to planner
//          treeSearchs[agentIdx]->motions2Tree(curr->getMotions(), agent_constraints);
         
//          /* replan for conflicting agent w/ new constraint */
//          Plan new_plan = lowLevelSearch(treeSearchs[agentIdx], agent_constraints, curr->getParent()->getPlan(), true);


//          // plan again
//          ob::PlannerStatus solved = treeSearchs[agentIdx]->ob::Planner::solve(planningTime_);
//          if (solved)
//          {
//             /* create new solution with updated traj. for conflicting agent */
//             oc::PathControl new_traj = static_cast<oc::PathControl &>
//                (*treeSearchs[agentIdx]->ob::Planner::getProblemDefinition()->getSolutionPath());
//             Plan new_plan;
//             for (int agent = 0; agent < mmpp_.size(); agent++)
//             {
//                if (agentIdx == agent)
//                   new_plan.push_back(new_traj);
//                else
//                   new_plan.push_back(curr->getParent()->getPlan()[agent]);
//             }
//             curr->updatePlanAndCost(new_plan);
//          }
//          else
//          {
//             // update the list of motions for next time
//             // save planner progress to node
//             std::vector<ConstraintRRT::Motion *> m;
//             treeSearchs[agentIdx]->dumpTree2Motions(m);
//             curr->fillMotions(m);
//             pq.emplace(*curr);
//             // printf("Failed again, should skip. \n");
//             continue;
//          }
      }
      // std::cout << "entering plan checker" << std::endl;
      std::vector<ConflictPtr> confs = mrmp_pdef_->getPlanValidator()->validatePlan(curr->getPlan());
      // std::cout << "done: " << confs.size() << std::endl;
      if (confs.empty())
      {
         solution = curr;
         break;
      }
      else if (shouldMerge_(conf_counter_, confs.front()->agent1Idx_, confs.front()->agent2Idx_))
      {
//          // we have decided to merge agents into a meta agent and restart the search
         OMPL_INFORM("%s: Too many conflicts exist between a pair of agents.", getName().c_str());
         OMPL_INFORM("%s: Composing a %s with a %s.", getName().c_str(),
            mrmp_pdef_->getInstance()->getRobots()[confs.front()->agent1Idx_]->getDynamicsModel().c_str(), 
            mrmp_pdef_->getInstance()->getRobots()[confs.front()->agent2Idx_]->getDynamicsModel().c_str());
         OMPL_ERROR("%s: This portion of code has not been updated since refactor. Aborting with failure.", getName().c_str());
         return {false, false};
         
//          // Instance* new_instance = composeSystem(conf[0].agent1, conf[0].agent2);
//          // if (new_instance == nullptr)
//          //    return base::PlannerStatus::INVALID_START;
         
//          // const std::vector<mrmp_problem> new_mmpp = multiAgentSetUp(new_instance);
//          // oc::K_CBS *p = new oc::K_CBS(new_mmpp); // K_CBS
//          // // subPlanners_.push_back(p);
//          // p->setInstance(new_instance);
//          // ob::PlannerPtr planner(p);
//          // OMPL_INFORM("Set-Up Complete");

//          // bool solved = planner->solve(ptc);
//          // auto stop = std::chrono::high_resolution_clock::now();
//          // auto duration = duration_cast<std::chrono::microseconds>(stop - start);
//          // solveTime_ = (duration.count() / 1000000.0);
//          // if (!solved)
//          //    return {solved, false};
//          // else
//          // {
//          //    solved = true;
//          //    for (int i = 0; i < mmpp_.size(); i++)
//          //       mmpp_[i].second->clearSolutionPaths();

//          //    // map new_problem solution to mmpp_
//          //    std::vector<PathControl> plan;
//          //    for (int a = 0; a < new_mmpp.size(); a++)
//          //    {
//          //       const std::string a_dyn = new_instance->getRobots()[a]->getDynamicsModel();
//          //       // printf("Here: %s \n", a_dyn.c_str());
//          //       if ((a_dyn == "Dynamic Car") || (a_dyn == "Kinematic Car") || (a_dyn == "Dynamic Unicycle"))
//          //       {
//          //          PathControl *mpath = new_mmpp[a].second->getSolutionPath()->as<PathControl>();
//          //          plan.push_back(*mpath);
//          //       }
//          //       else if (a_dyn == "Two Dynamic Cars")
//          //       {
//          //          // seperate the paths
//          //          PathControl *mpath = new_mmpp[a].second->getSolutionPath()->as<PathControl>();
//          //          plan.push_back(*mpath);
//          //          plan = decentralizeTrajectory(plan, new_instance);
//          //       }
//          //    }
//          //    for (int i = 0; i < mmpp_.size(); i++)
//          //    {
//          //       auto path(std::make_shared<PathControl>(plan[i]));
//          //       mmpp_[i].second->addSolutionPath(path, false, -1.0, w_->getAgents()[i]->getName());
//          //    }
//          //    OMPL_INFORM("%s: Found Solution in %0.3f seconds!", getName().c_str(), solveTime_ );
//          //    OMPL_INFORM("%s: Planning Complete.", getName().c_str());
//          //    return {solved, false};
         // }
      }
      else
      {
         pq.pop();
         /* extract conflict information */
         // for (auto itr = confs.begin(); itr != confs.end(); itr++) {
         //    std::cout << "Agent1: " << (*itr)->agent1Idx_ << std::endl;
         //    std::cout << "Agent2: " << (*itr)->agent2Idx_ << std::endl;
         //    std::cout << "Time: " << (*itr)->time_ << std::endl;
         // }
         ConstraintPtr agent1IdxConflicts = mrmp_pdef_->getPlanValidator()->createConstraint(curr->getPlan(), confs, confs.front()->agent1Idx_);
         ConstraintPtr agent2IdxConflicts = mrmp_pdef_->getPlanValidator()->createConstraint(curr->getPlan(), confs, confs.front()->agent2Idx_);
         std::vector<ConstraintPtr> new_constraints{agent1IdxConflicts, agent2IdxConflicts};

         OMPL_INFORM("Conflict between agents: (%d, %d)", confs.front()->agent1Idx_, confs.front()->agent2Idx_);

         for (int a = 0; a < 2; a++)
         {
            /* find a new trajectory that resolves the constraint */
            auto new_constraint = new_constraints[a];
            // auto *new_traj = new PathControl(mmpp_[conflicting_agents[a]].first);
            
            KCBSNode nxt;
            nxt.updateParent(curr);
            nxt.addConstraint(new_constraint);
            /* traverse conflict tree to get all agent constraints */
            std::vector<ConstraintPtr> agent_constraints{nxt.getConstraint()};
            const KCBSNode *nCpy = nxt.getParent();
            while (nCpy->getParent() != nullptr)
            {
               if (nCpy->getConstraint()->getAgent() == new_constraint->getAgent())
                  agent_constraints.emplace_back(nCpy->getConstraint());
               nCpy = nCpy->getParent();
            }
            /* replan for conflicting agent w/ new constraint */
            oc::PathControl *new_path = calcNewPath_(mrmp_pdef_->getRobotMotionPlanningProblemPtr(new_constraint->getAgent())->getPlanner(), agent_constraints);

            if (new_path) {
               Plan new_plan = curr->getPlan();
               new_plan[new_constraint->getAgent()] = *new_path;
               nxt.updatePlanAndCost(new_plan);
               pq.emplace(nxt);
            }
            else {
               /* Failed to find solution. 
                  Need to save data to node and put at back of queue.
               */
               OMPL_WARN("%s: Failed to Save planning data and add node to priority queue.", getName().c_str());
               // // save planner progress to node
               // std::vector<ConstraintRRT::Motion *> m;
               // treeSearchs[conflicting_agents[a]]->dumpTree2Motions(m);
               // n.fillMotions(m);
               // // add to queue with cost inf
               // pq.emplace(n);
            }
         }
      }
   }
   // /* end main algorithm -- begin ompl bookkeeping */
   auto stop = std::chrono::high_resolution_clock::now();
   auto duration = duration_cast<std::chrono::microseconds>(stop - start);
   solveTime_ = (duration.count() / 1000000.0);
   bool solved = false;
   if (solution == nullptr)
   {
      if (ptc == true)
         OMPL_INFORM("%s: No solution found due to time.", getName().c_str());
      return {solved, false};
   }
   else
   {
      // printf("%p\n", solution);
      // printf("Sol. size: %lu \n", solution->getPlan().size());
      solved = true;
      OMPL_INFORM("%s: Found Solution in %0.3f seconds!", getName().c_str(), 
         solveTime_);
      // /* add correct path to all problem instances */
      // for (int i = 0; i < mmpp_.size(); i++)
      // {
      //    auto path(std::make_shared<PathControl>(solution->getPlan()[i]));
      //    mmpp_[i].second->addSolutionPath(path, false, -1.0, mrmp_instance_->getRobots()[i]->getName());
      // }
      OMPL_INFORM("%s: Planning Complete.", getName().c_str());
      return {solved, false};
   }
}
