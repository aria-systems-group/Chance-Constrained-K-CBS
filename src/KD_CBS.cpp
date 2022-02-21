/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* The main source code of KD-CBS
*********************************************************************/
 
/* Author: Justin Kottinger */

#include "includes/KD_CBS.h"


namespace base = ompl::base;

// constructor
ompl::control::KD_CBS::KD_CBS(const std::vector<problem> mmpp) : 
   base::Planner(mmpp[0].first, "KD-CBS"), mmpp_{mmpp}
{
   // These params are for benchmarking -- ignore for now
   // Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");
   // Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates,
                                // "0,1");
}


// de-constructor (TO-DO)
ompl::control::KD_CBS::~KD_CBS()
{
    freeMemory();
}

// free all alocated memory
void ompl::control::KD_CBS::freeMemory()
{
   // not implemented yet
}

std::vector <Conflict> ompl::control::KD_CBS::validatePlan(Plan pl)
{
   // init the conflict set to be (possibly) filled
   std::vector <Conflict> c;
   
   // get minimum step size
   const auto *si = static_cast<const SpaceInformation *>(pl[0].getSpaceInformation().get());
   const double minStepSize = si->getPropagationStepSize();

   // interpolate all trajectories to the same discretization
   for (int i = 0; i < pl.size(); i++)
      pl[i].interpolate();
   
   // printf("done interpoloating.\n");
   // get longest trajectory -- i.e. max states
   int maxStates = 0;
   for (int i = 0; i < pl.size(); i++)
   {
      if (maxStates < pl[i].getStateCount())
         maxStates = pl[i].getStateCount();
   }
   
   // iterate through all the states
   for (int k = 0; k < maxStates; k++)
   {
      // for each k, get all the (valid) states for each valid agent
      std::vector<base::State *> validStatesAtK;
      std::vector<int> validAgentsIdx;
      for (int i = 0; i < pl.size(); i++)
      {
         if (k < pl[i].getStateCount())
         {
            validStatesAtK.push_back(pl[i].getState(k));
            validAgentsIdx.push_back(i);
         }
         else
         {
            validStatesAtK.push_back(pl[i].getStates().back());
            validAgentsIdx.push_back(i);
         }
      }
      // for each valid agent at k, create agent shape
      std::vector<std::pair<int, polygon>> shapes;
      for (int j = 0; j < validAgentsIdx.size(); j++)
      {
         // get agent information from world
         const Agent *a = w_->getAgents()[validAgentsIdx[j]];
         if ((a->getDynamics() == "Dynamic Car") || (a->getDynamics() == "Kinematic Car") ||
            (a->getDynamics() == "Dynamic Unicycle"))
         {
            const double carWidth = a->getShape()[0];
            const double carHeight = a->getShape()[1];
            // extract points from state object
            auto compState = validStatesAtK[j]->as<ob::CompoundStateSpace::StateType>();
            auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
            const double cx = xyState->values[0];
            const double cy = xyState->values[1];
            const double theta = compState->as<ob::SO2StateSpace::StateType>(1)->value;

            // turn (x,y, theta), width, length to a polygon object
            // borrowed from conflictChecking
            // TOP RIGHT VERTEX:
            const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
            const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
            std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
            // TOP LEFT VERTEX:
            const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
            const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
            std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
            // BOTTOM LEFT VERTEX:
            const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
            const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
            std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
            // BOTTOM RIGHT VERTEX:
            const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
            const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
            std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);
            // convert to string for easy initializataion
            std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
            polygon agent;
            boost::geometry::read_wkt(points,agent);
            std::pair<int, polygon> pair{validAgentsIdx[j], agent};
            shapes.push_back(pair);
         }
         else if (a->getDynamics() == "Two Dynamic Cars")
         {
            // printf("size of valid agents: %lu \n", validAgentsIdx.size());
            // printf("size of valid states: %lu \n", validStatesAtK.size());
            // printf("j: %i \n", j);
            // extract points from state object
            auto compState = validStatesAtK[j]->as<ob::CompoundStateSpace::StateType>();
            // printf("got composed state.\n");
            // add two shapes (one for each vehicle)
            for (int agnt = 0; agnt < 2; agnt++)
            {
               auto xyState = compState->as<
                    ob::RealVectorStateSpace::StateType>(2*agnt + 0);
               // printf("got xyState state.\n");
               const double cx = xyState->values[0];
               const double cy = xyState->values[1];
               const double theta = compState->as<
                  ob::SO2StateSpace::StateType>(2*agnt + 1)->value;
               // printf("got got theta.\n");

               // Get important params from car object
               const double carWidth = a->getShape()[0];
               const double carHeight = a->getShape()[1];

               // printf("got car shapes.\n");

               // turn (x,y, theta), width, length to a polygon object
               // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
               // TOP RIGHT VERTEX:
               const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
               const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
               std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
               // TOP LEFT VERTEX:
               const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
               const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
               std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
               // BOTTOM LEFT VERTEX:
               const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
               const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
               std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
               // BOTTOM RIGHT VERTEX:
               const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
               const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
               std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);

               // convert to string for easy initializataion
               std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
               polygon agent;
               boost::geometry::read_wkt(points,agent);

               std::pair<int, polygon> pair{validAgentsIdx[j], agent};
               shapes.push_back(pair);
            }
         }
         else
         {
            OMPL_ERROR("Validate Function not implemented for %s.", a->getDynamics().c_str());
         }
      }
      // printf("shapes added successfully.\n");
      // disjoint check the shapes for a collision
      for (int ai = 0; ai < shapes.size(); ai++)
      {
         for (int aj = 0; aj < shapes.size(); aj++)
         {
            if (shapes[ai].first != shapes[aj].first)
            {
               if (! boost::geometry::disjoint(shapes[ai].second, shapes[aj].second))
               {
                  // printf("Conflict found.\n");
                  // agent ai and aj are in conflict, only care about those now
                  Conflict conf{shapes[ai].first, shapes[aj].first, 
                     shapes[ai].second, shapes[aj].second, (k * minStepSize)};
                  c.push_back(conf);
                  bool inConflict = true;
                  // printf("Created first conflict.\n");
                  while (inConflict)
                  {
                     k++;
                     // for each k, get the (valid) states for each valid agent
                     std::vector<base::State *> validStatesAtK;
                     if (k < pl[shapes[ai].first].getStateCount())
                        validStatesAtK.push_back(pl[shapes[ai].first].getState(k));
                     else
                        validStatesAtK.push_back(pl[shapes[ai].first].getStates().back());

                     if (k < pl[shapes[aj].first].getStateCount())
                        validStatesAtK.push_back(pl[shapes[aj].first].getState(k));
                     else
                        validStatesAtK.push_back(pl[shapes[aj].first].getStates().back());


                     if ((mmpp_[shapes[ai].first].first->
                           equalStates(validStatesAtK[0], pl[shapes[ai].first].getStates().back())) && 
                        (mmpp_[shapes[aj].first].first->
                           equalStates(validStatesAtK[1], pl[shapes[aj].first].getStates().back())))
                     {
                        inConflict = false;
                     }
                     else
                     {
                        std::vector<polygon> polys;
                        for (int i = 0; i < 2; i++)
                        {
                           int currIdx = 0;
                           if (i == 0)
                              currIdx = shapes[ai].first;
                           else
                              currIdx = shapes[aj].first;
                        
                           // get first agent information from world
                           const Agent *a = w_->getAgents()[currIdx];
                           if ((a->getDynamics() == "Dynamic Car") || (a->getDynamics() == "Kinematic Car") ||
                              (a->getDynamics() == "Dynamic Unicycle"))
                           {
                              const double carWidth = a->getShape()[0];
                              const double carHeight = a->getShape()[1];
                              // extract points from state object
                              auto compState = validStatesAtK[i]->as<ob::CompoundStateSpace::StateType>();
                              auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
                              const double cx = xyState->values[0];
                              const double cy = xyState->values[1];
                              const double theta = compState->as<ob::SO2StateSpace::StateType>(1)->value;
                              // turn (x,y, theta), width, length to a polygon object
                              // borrowed from conflictChecking
                              // TOP RIGHT VERTEX:
                              const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
                              const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
                              std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
                              // TOP LEFT VERTEX:
                              const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
                              const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
                              std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
                              // BOTTOM LEFT VERTEX:
                              const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
                              const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
                              std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
                              // BOTTOM RIGHT VERTEX:
                              const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
                              const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
                              std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);
                              // convert to string for easy initializataion
                              std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
                              polygon agent;
                              boost::geometry::read_wkt(points, agent);
                              polys.push_back(agent);
                           }
                           else if (a->getDynamics() == "Two Dynamic Cars")
                           {
                              // extract points from state object
                              auto compState = validStatesAtK[i]->as<ob::CompoundStateSpace::StateType>();
                              // add two shapes (one for each vehicle)
                              for (int agnt = 0; agnt < 2; agnt++)
                              {
                                 auto xyState = compState->as<
                                      ob::RealVectorStateSpace::StateType>(2*agnt + 0);
                                 const double cx = xyState->values[0];
                                 const double cy = xyState->values[1];
                                 const double theta = compState->as<
                                    ob::SO2StateSpace::StateType>(2*agnt + 1)->value;

                                 // Get important params from car object
                                 const double carWidth = a->getShape()[0];
                                 const double carHeight = a->getShape()[1];

                                 // turn (x,y, theta), width, length to a polygon object
                                 // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
                                 // TOP RIGHT VERTEX:
                                 const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
                                 const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
                                 std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
                                 // TOP LEFT VERTEX:
                                 const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
                                 const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
                                 std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
                                 // BOTTOM LEFT VERTEX:
                                 const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
                                 const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
                                 std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
                                 // BOTTOM RIGHT VERTEX:
                                 const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
                                 const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
                                 std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);
                                 // convert to string for easy initializataion
                                 std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
                                 polygon agent;
                                 boost::geometry::read_wkt(points,agent);
                                 polys.push_back(agent);
                              }
                           }
                        }
                        bool addedConflict = false;
                        for (int t1 = 0; t1 < polys.size(); t1++)
                        {
                           for (int t2 = 0; t2 < polys.size(); t2++)
                           {
                              if (t1 != t2)
                              {
                                 // check if agetnai and agent aj are disjoint at next time step
                                 if (! boost::geometry::disjoint(polys[t1], polys[t2]))
                                 {
                                    // agent ai and aj are in conflict, only care about those now
                                    Conflict conf{shapes[ai].first, shapes[aj].first, 
                                       polys[t1], polys[t2], (k * minStepSize)};
                                    c.push_back(conf);
                                    addedConflict = true;
                                 }
                              }
                           }
                        }
                        if (!addedConflict)
                           inConflict = false;
                     }
                  }
                  // check if states we just checked were both the end
                  // if so terminate the search.
                  // printf("finished with conflicts.\n");
                  return c;
               }
            }
         }
      }
   }
   // printf("no conflicts found.\n");
   return c;
}


bool ompl::control::KD_CBS::shouldMerge(
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

World* ompl::control::KD_CBS::composeSystem(const int agentIdx1, const int agentIdx2)
{
   std::vector<problem> x;
   /* first, change the world params to match new problem.
      world object has a vector of Agents. need to replace the two 
      at agent1 and agent2 idx and merge them into a single agent*/

   w_->printAgents();

   // create new world
   World *new_world = new World();
   // give map dimensions
   new_world->setWorldDimensions(w_->getWorldDimensions()[0], 
      w_->getWorldDimensions()[1]);
   // provide obstacles
   for (auto o: w_->getObstacles())
      new_world->addObstacle(o);
   // provide agents (except the ones that will be merged)
   for(int a = 0; a < w_->getAgents().size(); a++)
   {
      if ((a != agentIdx1) && (a!= agentIdx2))
         new_world->addAgent(w_->getAgents()[a]);
   }

   /* add new agent */
   if ((w_->getAgents()[agentIdx1]->getDynamics() == "Dynamic Car") && 
       (w_->getAgents()[agentIdx2]->getDynamics() == "Dynamic Car"))
   {
      OMPL_INFORM("%s: Creating a %s model", getName().c_str(), "Two Dynamic Cars");

      // std::vector<double> s, std::vector<double> g
      std::string new_name = "agent" + std::to_string(new_world->getAgents().size());
      std::vector<double> new_start{
         w_->getAgents()[agentIdx1]->getStartLocation()[0], 
         w_->getAgents()[agentIdx1]->getStartLocation()[1],
         w_->getAgents()[agentIdx2]->getStartLocation()[0],
         w_->getAgents()[agentIdx2]->getStartLocation()[1]};
      std::vector<double> new_goal{
         w_->getAgents()[agentIdx1]->getGoalLocation()[0], 
         w_->getAgents()[agentIdx1]->getGoalLocation()[1],
         w_->getAgents()[agentIdx2]->getGoalLocation()[0],
         w_->getAgents()[agentIdx2]->getGoalLocation()[1]};

      Agent* new_agent = new Agent(new_name, "Two Dynamic Cars", 
         w_->getAgents()[0]->getShape(), new_start, new_goal);

      new_world->addAgent(new_agent);

      new_world->printAgents();
   }
   else
   {
      OMPL_ERROR("%s: unable to compose %s and %s. Implementation must be extended.",
         getName().c_str(), w_->getAgents()[agentIdx1]->getDynamics().c_str(), 
         w_->getAgents()[agentIdx2]->getDynamics().c_str());
      return nullptr;
   }

   return new_world;
}

// the main algorithm
base::PlannerStatus ompl::control::KD_CBS::solve(const base::PlannerTerminationCondition &ptc)
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

   /* create instances of low-level planner */
   std::vector<oc::constraintRRT*> treeSearchs;
   for (int a = 0; a < mmpp_.size(); a++)
   {
      constraintRRT* planner = new constraintRRT(mmpp_[a].first);
      planner->setProblemDefinition(mmpp_[a].second);
      planner->provideAgent(w_->getAgents()[a]);
      if (w_->getAgents()[a]->getDynamics() == "Two Dynamic Cars")
         planner->isCentralized(true);
      planner->setup();
      treeSearchs.push_back(planner);
   }

   /* initialize disjoint sets */
   std::vector< std::pair< std::pair<int, int>, int> > disjoint_set;
   for (int a1 = 0; a1 < mmpp_.size(); a1++)
   {
      for (int a2 = 0; a2 < mmpp_.size(); a2++)
      {
         if (a1 != a2)
         {
            // does (a1, a2) exist in disjoint_set?
            bool add = true;
            for (int i = 0; i < disjoint_set.size(); i++)
            {
               std::pair<int, int> curr_pair = disjoint_set[i].first;
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
               disjoint_set.push_back(conflict_count);
            }
         }
      }
   }

   /* initialize priority queue */ 
   std::priority_queue<conflictNode, std::vector<conflictNode>, Compare> pq;

   /* begin planning -- timing should start after this statement */
   OMPL_INFORM("%s: Starting planning. ", getName().c_str());
   auto start = std::chrono::high_resolution_clock::now();

   /* create initial solution */
   Plan root_plan;
   for (auto p: treeSearchs)
   {
      ob::PlannerStatus solved = p->ob::Planner::solve(planningTime_);
      while (!solved && !ptc)
         solved = p->ob::Planner::solve(planningTime_);

      /* store initial trajectory */
      if (solved)
      {
         oc::PathControl traj = static_cast<oc::PathControl &>
         (*p->ob::Planner::getProblemDefinition()->getSolutionPath());
         root_plan.push_back(traj);
      }
      p->clear();
   }

   /* create root node */
   conflictNode rootNode{};
   if (root_plan.size() == mmpp_.size())
   {
      rootNode.updatePlanAndCost(root_plan);
      pq.emplace(rootNode);
   }
 
   /* initialize solution */
   conflictNode *solution = nullptr;
   while (ptc == false && !pq.empty())
   {
      // printf("At top of loop \n");
      /* find loswest cost in Queue */
      // std::shared_ptr<conflictNode> curr = std::make_shared<conflictNode>(pq.top());
      // auto *curr = new conflictNode(pq.top());
      // std::shared_ptr<conflictNode> c = std::make_shared<conflictNode>(curr);
      auto *curr = new conflictNode(pq.top());

      /* if cost==inf need to try to re-plan node */
      if (curr->getCost() == std::numeric_limits<double>::infinity())
      {
         pq.pop();
         // printf("Now Here: %lu \n", curr->getMotions().size());
         // update planner information
         const int agentIdx = curr->getConstraint()->getAgent();
         // get constraints
         std::vector<const Constraint*> agent_constraints{curr->getConstraint()};
         const conflictNode *nCpy = curr->getParent();
         while (nCpy->getConstraint() != nullptr)
         {
            if (nCpy->getConstraint()->getAgent() == agentIdx)
               agent_constraints.push_back(nCpy->getConstraint());
            nCpy = nCpy->getParent();
         }
         // add constraints and motions to planner
         treeSearchs[agentIdx]->motions2Tree(curr->getMotions(), agent_constraints);
         // plan again
         ob::PlannerStatus solved = treeSearchs[agentIdx]->ob::Planner::solve(planningTime_);
         if (solved)
         {
            /* create new solution with updated traj. for conflicting agent */
            oc::PathControl new_traj = static_cast<oc::PathControl &>
               (*treeSearchs[agentIdx]->ob::Planner::getProblemDefinition()->getSolutionPath());
            Plan new_plan;
            for (int agent = 0; agent < mmpp_.size(); agent++)
            {
               if (agentIdx == agent)
                  new_plan.push_back(new_traj);
               else
                  new_plan.push_back(curr->getParent()->getPlan()[agent]);
            }
            curr->updatePlanAndCost(new_plan);
         }
         else
         {
            // update the list of motions for next time
            // save planner progress to node
            std::vector<constraintRRT::Motion *> m;
            treeSearchs[agentIdx]->dumpTree2Motions(m);
            curr->fillMotions(m);
            pq.emplace(*curr);
            // printf("Failed again, should skip. \n");
            continue;
         }
      }
      // printf("continuing. \n");
      std::vector <Conflict> conf = validatePlan(curr->getPlan());
      // printf("conf: (%i, %i) \n", conf[0].agent1, conf[0].agent2);
      if (conf.empty())
      {
         // printf("Sol. Size: %lu \n", curr->getPlan().size());
         solution = curr; //curr.get();
         break;
      }
      else if (shouldMerge(disjoint_set, conf[0].agent1, conf[0].agent2))
      {
         // we have decided to merge agents into a meta agent and restart the search
         OMPL_INFORM("%s: Too many conflicts exist between a pair of agents.", getName().c_str());
         OMPL_INFORM("%s: Composing a %s with a %s.", getName().c_str(),
            w_->getAgents()[conf[0].agent1]->getDynamics().c_str(), 
            w_->getAgents()[conf[0].agent2]->getDynamics().c_str());
         
         World* new_world  = composeSystem(conf[0].agent1, conf[0].agent2);
         if (new_world == nullptr)
            return base::PlannerStatus::INVALID_START;
         
         const std::vector<problem> new_mmpp = multiAgentSetUp(new_world);
         oc::KD_CBS *p = new oc::KD_CBS(new_mmpp); // K_CBS
         // subPlanners_.push_back(p);
         p->setWorld(new_world);
         ob::PlannerPtr planner(p);
         OMPL_INFORM("Set-Up Complete");
         // std::cout << "Setup Complete. Press ENTER to plan: ";
         // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
         // auto curr = std::chrono::high_resolution_clock::now();
         // auto curr_dur = duration_cast<std::chrono::microseconds>(curr - start);
         // double time_left = 600.0 - (curr_dur.count() / 1000000.0);

         bool solved = planner->solve(ptc);
         auto stop = std::chrono::high_resolution_clock::now();
         auto duration = duration_cast<std::chrono::microseconds>(stop - start);
         solveTime_ = (duration.count() / 1000000.0);
         if (!solved)
            return {solved, false};
         else
         {
            solved = true;
            for (int i = 0; i < mmpp_.size(); i++)
               mmpp_[i].second->clearSolutionPaths();

            // map new_problem solution to mmpp_
            std::vector<PathControl> plan;
            for (int a = 0; a < new_mmpp.size(); a++)
            {
               const std::string a_dyn = new_world->getAgents()[a]->getDynamics();
               // printf("Here: %s \n", a_dyn.c_str());
               if ((a_dyn == "Dynamic Car") || (a_dyn == "Kinematic Car") || (a_dyn == "Dynamic Unicycle"))
               {
                  PathControl *mpath = new_mmpp[a].second->getSolutionPath()->as<PathControl>();
                  plan.push_back(*mpath);
               }
               else if (a_dyn == "Two Dynamic Cars")
               {
                  // seperate the paths
                  PathControl *mpath = new_mmpp[a].second->getSolutionPath()->as<PathControl>();
                  plan.push_back(*mpath);
                  plan = decentralizeTrajectory(plan, new_world);
               }
            }
            for (int i = 0; i < mmpp_.size(); i++)
            {
               auto path(std::make_shared<PathControl>(plan[i]));
               mmpp_[i].second->addSolutionPath(path, false, -1.0, w_->getAgents()[i]->getName());
            }
            OMPL_INFORM("%s: Found Solution in %0.3f seconds!", getName().c_str(), solveTime_ );
            OMPL_INFORM("%s: Planning Complete.", getName().c_str());
            return {solved, false};
         }
      }
      else
      {
         pq.pop();
         /* extract conflict information*/
         const std::vector<int> conflicting_agents
            {conf.front().agent1, conf.front().agent2};
         std::vector<std::vector<polygon>> conflicting_polys{ {}, {} };
         std::vector<double> conflicting_times{};
         /* fill time and polygon vectors*/
         for (Conflict c: conf)
         {
            conflicting_times.push_back(c.time);
            conflicting_polys[0].push_back(c.p2);
            conflicting_polys[1].push_back(c.p1);
         }

         for (int a = 0; a < 2; a++)
         {
            /* init new constraint and trajectory to resolve conflict */
            auto *new_constraint = new Constraint(conflicting_polys[a], 
               conflicting_times, conflicting_agents[a]);
            auto *new_traj = new PathControl(mmpp_[conflicting_agents[a]].first);
            
            conflictNode n{};
            n.updateParent(curr);
            n.addConstraint(new_constraint);
            // std::cout << "replanning required" << std::endl;
            /* traverse conflict tree to get all agent constraints */
            std::vector<const Constraint*> agent_constraints{n.getConstraint()};
            const conflictNode *nCpy = n.getParent();
            while (nCpy->getConstraint() != nullptr)
            {
               if (nCpy->getConstraint()->getAgent() == conflicting_agents[a])
                  agent_constraints.push_back(nCpy->getConstraint());
               nCpy = nCpy->getParent();
            }
            /* replan for conflicting agent w/ new constraint */
            treeSearchs[conflicting_agents[a]]->updateConstraints(agent_constraints);
            ob::PlannerStatus solved = treeSearchs[conflicting_agents[a]]->ob::Planner::solve(planningTime_);
            if (solved)
            {
               /* create new solution with updated traj. for conflicting agent */
               oc::PathControl new_traj = static_cast<oc::PathControl &>
                  (*treeSearchs[conflicting_agents[a]]->ob::Planner::getProblemDefinition()->getSolutionPath());
               Plan new_plan;
               for (int agent = 0; agent < mmpp_.size(); agent++)
               {
                  if (conflicting_agents[a] == agent)
                     new_plan.push_back(new_traj);
                  else
                     new_plan.push_back(n.getParent()->getPlan()[agent]);
               }
               n.updatePlanAndCost(new_plan);
               pq.emplace(n);
            }
            else
            {
               /* 
               failed to find solution. 
               Need to save data to node and put at back of queue 
               */
               // printf("HERE: %d \n", a);
               // save planner progress to node
               std::vector<constraintRRT::Motion *> m;
               treeSearchs[conflicting_agents[a]]->dumpTree2Motions(m);
               n.fillMotions(m);
               // printf("Size of motions in n: %lu \n", n.getMotions().size());

               // add to queue with cost inf
               // printf("Address of n: %p \n", &n);
               pq.emplace(n);
            }
         }
      }
   }
   /* end main algorithm -- begin ompl bookkeeping */
   auto stop = std::chrono::high_resolution_clock::now();
   auto duration = duration_cast<std::chrono::microseconds>(stop - start);
   solveTime_ = (duration.count() / 1000000.0);
   bool solved = false;
   /* clean up */
   for (auto p: treeSearchs)
   {
      p->clear();
      delete p;
   }
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
      /* add correct path to all problem instances */
      for (int i = 0; i < mmpp_.size(); i++)
      {
         auto path(std::make_shared<PathControl>(solution->getPlan()[i]));
         mmpp_[i].second->addSolutionPath(path, false, -1.0, w_->getAgents()[i]->getName());
      }
      OMPL_INFORM("%s: Planning Complete.", getName().c_str());
      return {solved, false};
   }
}
