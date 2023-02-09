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
   	prop_step_size_ = mrmp_pdef_->getSystemStepSize();
   	for (auto itr = mrmp_info.begin() + 1; itr != mrmp_info.end(); itr++) {
   	   	const oc::SpaceInformationPtr siPtr = (*itr)->getSpaceInformation();
   	   	const double dt = siPtr->getPropagationStepSize();
   	   	if (dt != prop_step_size_) {
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
   	for (int a1 = 0; a1 < mrmp_info.size(); a1++) {
   	   	for (int a2 = 0; a2 < mrmp_info.size(); a2++) {
   	   	   if (a1 != a2) {
   	   	      	// does (a1, a2) exist in conf_counter_?
   	   	      	bool add = true;
   	   	      	for (int i = 0; i < conf_counter_.size(); i++) {
   	   	      	   	std::pair<int, int> curr_pair = conf_counter_[i].first;
   	   	      	   	if (curr_pair.first == a1 && curr_pair.second == a2)
   	   	      	   		add = false;
   	   	      	   	else if (curr_pair.first == a2 && curr_pair.second == a1)
   	   	      	   		add = false;
   	   	      	   	if (add == false)
   	   	      	   		break;
   	   	      	}
   	   	      	if (add) {
   	   	      	   	std::pair<int, int> agnt_pair{a1, a2};
   	   	      	   	std::pair< std::pair<int, int>, int > conflict_count{agnt_pair, 0};
   	   	      	   	conf_counter_.push_back(conflict_count);
   	   	      	}
   	   	   }
   	   	}
   	}
   	ready_ = true;
}

ompl::control::KCBS::~KCBS()
{
	/* Free all allocated memory */
	freeMemory_();
}

void ompl::control::KCBS::freeMemory_()
{
	/* free all alocated memory */
	OMPL_WARN("%s: freeMemory_ called but not yet implemented. Possible memory leak.", getName().c_str());
}

oc::PathControl* ompl::control::KCBS::calcNewPath_(ConstraintRespectingPlannerPtr planner, std::vector<ConstraintPtr> constraints)
{
	/* Update the constraints for planner and attempt to resolve them in mp_comp_time_ seconds */
	/* If replanning was successful, return new path. Otherwise, return nullptr */
   	oc::PathControl *traj = nullptr;
   	planner->updateConstraints(constraints);
   	ob::PlannerStatus solved = planner->solve(mp_comp_time_);
   	if (solved==ob::PlannerStatus::EXACT_SOLUTION) {
   	   	OMPL_INFORM("%s: Successfully Replanned.", getName().c_str());
   	   	/* create new solution with updated traj. for conflicting agent */
   	   	traj = planner->getProblemDefinition()->getSolutionPath()->as<oc::PathControl>();
   	   	return traj;
   	}
   	else {
   	   	OMPL_INFORM("Failed to replan.");
   	   	return traj; // empty
   	}
}

bool ompl::control::KCBS::shouldMerge_(std::vector< std::pair< std::pair<int, int>, int> > &conf_cntr, const int agent1, const int agent2)
{
	/* Update the global conflict counter */
   	for (int i = 0; i < conf_cntr.size(); i++) {
   	   	if ((conf_cntr[i].first.first == agent1) && (conf_cntr[i].first.second == agent2)) {
   	   	   	conf_cntr[i].second = conf_cntr[i].second + 1;
   	   	   	break;
   	   	}
   	   	else if ((conf_cntr[i].first.first == agent2) && (conf_cntr[i].first.second == agent1)) {
   	   	   	conf_cntr[i].second = conf_cntr[i].second + 1;
   	   	   	break;
   	   	}
   	}
   	/* Check if the merge bound was triggered */
   	for (int i = 0; i < conf_cntr.size(); i++) {
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
	/* Be sure that K-CBS was set-up */
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

   	/* create initial solution */
   	Plan root_plan;
   	for (auto itr = low_level_planners_.begin(); itr != low_level_planners_.end(); itr++) {
   	   	ob::PlannerStatus solved = (*itr)->solve(mp_comp_time_);
   	   	while (solved!=ob::PlannerStatus::EXACT_SOLUTION && !ptc){
   	   	   	solved = (*itr)->solve(mp_comp_time_);
   	   	   	if (solved == base::PlannerStatus::INVALID_START) {
   	   	   	   	return base::PlannerStatus::INVALID_START;
   	   	   	}
   	   	}
   	   	/* store initial trajectory */
   	   	if (solved) {
   	   	   	oc::PathControl* traj = (*itr)->getProblemDefinition()->getSolutionPath()->as<oc::PathControl>();
   	   	   	root_plan.push_back(*traj);
   	   	}
   	   	(*itr)->clear();
   	}

   	/* create root node */
   	KCBSNode rootNode;
   	if (root_plan.size() == all_mp_pdefs.size()) {
   	   	rootNode.updatePlanAndCost(root_plan);
   	   	pq.emplace(rootNode);
   	}
 
   	/* initialize solution */
   	KCBSNode *solution = nullptr;
   	while (ptc == false && !pq.empty()) {
    	/* Get the lowest cost in priority queue */
      	auto curr = new KCBSNode(pq.top());

      	/* If the current K-CBS Node does not contain a finite-length solution, must replan with existing tree */
      	if (curr->getCost() == std::numeric_limits<double>::infinity()) {
      		pq.pop();
         	std::vector<ConstraintPtr> agent_constraints{curr->getConstraint()};
        	const KCBSNode *nCpy = curr->getParent();
        	while (nCpy->getParent() != nullptr) {
           		if (nCpy->getConstraint()->getConstrainedAgent() == curr->getConstraint()->getConstrainedAgent())
              		agent_constraints.emplace_back(nCpy->getConstraint());
           		nCpy = nCpy->getParent();
        	}
        	/* replan for conflicting agent w/ new constraint */
        	ConstraintRespectingPlannerPtr p = curr->getPlanner();
        	oc::PathControl *new_path = calcNewPath_(p, agent_constraints);

        	if (new_path) {
        		/* Create new node and add it to the queue */
        		KCBSNode nxt;
            	nxt.updateParent(curr);
            	nxt.addConstraint(curr->getConstraint());
            	Plan new_plan = curr->getParent()->getPlan();
            	new_plan[curr->getConstraint()->getConstrainedAgent()] = *new_path;
            	nxt.updatePlanAndCost(new_plan);
            	pq.emplace(nxt);
            }
            else {
            	/* Failed to find solution. Must copy data to new node and put at back of queue */
            	KCBSNode nxt;
            	nxt.updateParent(curr->getParent());
            	nxt.addConstraint(curr->getConstraint());
				nxt.savePlanner(curr->getPlanner());
            	pq.emplace(nxt);
            	mrmp_pdef_->replacePlanner(p, curr->getConstraint()->getConstrainedAgent());
            	pq.emplace(nxt);
            }
      	}
      	else {
      		/* Current K-CBS Node has a finite-length plan */
      		/* Simulate the plan in search of conflicts. If no conflicts arrise, return correct solution */
      		std::vector<ConflictPtr> confs = mrmp_pdef_->getPlanValidator()->validatePlan(curr->getPlan());
    		if (confs.empty()) {
        	 	solution = curr;
        	 	break;
      		}
      		/* Plan contains conflicts that triggered the merge bound. K-CBS must merge the two conflicting robots */
      		else if (shouldMerge_(conf_counter_, confs.front()->agent1Idx_, confs.front()->agent2Idx_)) {
        	 	OMPL_INFORM("%s: Too many conflicts exist between a pair of agents.", getName().c_str());
        	 	OMPL_INFORM("%s: Composing a %s with a %s.", getName().c_str(),
        	    	mrmp_pdef_->getInstance()->getRobots()[confs.front()->agent1Idx_]->getDynamicsModel().c_str(), 
        	    	mrmp_pdef_->getInstance()->getRobots()[confs.front()->agent2Idx_]->getDynamicsModel().c_str());
        	 	OMPL_ERROR("%s: The Merge block of K-CBS has not been updated since refactor. Aborting with failure.", getName().c_str());
        	 	return {false, false};
         
        		// Instance* new_instance = composeSystem(conf[0].agent1, conf[0].agent2);
        		// if (new_instance == nullptr)
        		//    return base::PlannerStatus::INVALID_START;
         
        		// const std::vector<mrmp_problem> new_mmpp = multiAgentSetUp(new_instance);
        		// oc::K_CBS *p = new oc::K_CBS(new_mmpp); // K_CBS
        		// // subPlanners_.push_back(p);
        		// p->setInstance(new_instance);
        		// ob::PlannerPtr planner(p);
        		// OMPL_INFORM("Set-Up Complete");

        		// bool solved = planner->solve(ptc);
        		// auto stop = std::chrono::high_resolution_clock::now();
        		// auto duration = duration_cast<std::chrono::microseconds>(stop - start);
        		// solveTime_ = (duration.count() / 1000000.0);
        		// if (!solved)
        		//    return {solved, false};
        		// else
        		// {
        		//    solved = true;
        		//    for (int i = 0; i < mmpp_.size(); i++)
        		//       mmpp_[i].second->clearSolutionPaths();

        		//    // map new_problem solution to mmpp_
        		//    std::vector<PathControl> plan;
        		//    for (int a = 0; a < new_mmpp.size(); a++)
        		//    {
        		//       const std::string a_dyn = new_instance->getRobots()[a]->getDynamicsModel();
        		//       // printf("Here: %s \n", a_dyn.c_str());
        		//       if ((a_dyn == "Dynamic Car") || (a_dyn == "Kinematic Car") || (a_dyn == "Dynamic Unicycle"))
        		//       {
        		//          PathControl *mpath = new_mmpp[a].second->getSolutionPath()->as<PathControl>();
        		//          plan.push_back(*mpath);
        		//       }
        		//       else if (a_dyn == "Two Dynamic Cars")
        		//       {
        		//          // seperate the paths
        		//          PathControl *mpath = new_mmpp[a].second->getSolutionPath()->as<PathControl>();
        		//          plan.push_back(*mpath);
        		//          plan = decentralizeTrajectory(plan, new_instance);
        		//       }
        		//    }
        		//    for (int i = 0; i < mmpp_.size(); i++)
        		//    {
        		//       auto path(std::make_shared<PathControl>(plan[i]));
        		//       mmpp_[i].second->addSolutionPath(path, false, -1.0, w_->getAgents()[i]->getName());
        		//    }
        		//    OMPL_INFORM("%s: Found Solution in %0.3f seconds!", getName().c_str(), solveTime_ );
        		//    OMPL_INFORM("%s: Planning Complete.", getName().c_str());
        		//    return {solved, false};
      		}
      		/* Plan contains conflicts. K-CBS must remove the current node from the priority queue and attempt to expand from it */
      		else {
        	 	pq.pop();
        	 	/* extract conflict information */
        	 	ConstraintPtr agent1IdxConstraint = mrmp_pdef_->getPlanValidator()->createConstraint(curr->getPlan(), confs, confs.front()->agent1Idx_);
        	 	ConstraintPtr agent2IdxConstraint = mrmp_pdef_->getPlanValidator()->createConstraint(curr->getPlan(), confs, confs.front()->agent2Idx_);
        	 	std::vector<ConstraintPtr> new_constraints{agent1IdxConstraint, agent2IdxConstraint};

         		OMPL_INFORM("Conflict between agents: (%d, %d) at time range [%0.1f, %0.1f]", 
         			confs.front()->agent1Idx_, confs.front()->agent2Idx_, confs.front()->timeStep_ * prop_step_size_, confs.back()->timeStep_ * prop_step_size_);

         		for (int a = 0; a < 2; a++)
         		{
         			/* Prepare one child K-CBS Node for every new constraint */
         			auto new_constraint = new_constraints[a];
            		KCBSNode nxt;
            		nxt.updateParent(curr);
            		nxt.addConstraint(new_constraint);
            	
            		/* Traverse conflict tree to get all agent constraints */
            		std::vector<ConstraintPtr> agent_constraints{nxt.getConstraint()};
            		const KCBSNode *nCpy = nxt.getParent();
            		while (nCpy->getParent() != nullptr)
            		{
            		   if (nCpy->getConstraint()->getConstrainedAgent() == new_constraint->getConstrainedAgent())
            		      agent_constraints.emplace_back(nCpy->getConstraint());
            		   nCpy = nCpy->getParent();
            		}
            	
            		/* Replan for conflicting agent w/ new constraint */
            		ConstraintRespectingPlannerPtr p = mrmp_pdef_->getRobotMotionPlanningProblemPtr(new_constraint->getConstrainedAgent())->getPlanner();
            		oc::PathControl *new_path = calcNewPath_(p, agent_constraints);

            		if (new_path) {
            			/* Create new node and add it to the queue */
            			Plan new_plan = curr->getPlan();
            			new_plan[new_constraint->getConstrainedAgent()] = *new_path;
            			nxt.updatePlanAndCost(new_plan);
            			pq.emplace(nxt);
            		}
            		else {
            			/* Failed to find solution. Save the planner inside node, reset the global planner, and add to end of queue */
            			nxt.savePlanner(p);
            			mrmp_pdef_->replacePlanner(p, new_constraint->getConstrainedAgent());
            			pq.emplace(nxt);
            		}
         		}
      		}
      	}
   	}
   	/* End of main loop. If possible, add solutions to every MotionPlanningProblem */
   	auto stop = std::chrono::high_resolution_clock::now();
   	auto duration = duration_cast<std::chrono::microseconds>(stop - start);
   	solveTime_ = (duration.count() / 1000000.0);
   	bool solved = false;
   	if (solution == nullptr) {
   	 	if (ptc == true)
   	 	   OMPL_INFORM("%s: No solution found due to time.", getName().c_str());
   	 	return {solved, false};
   	}
   	else {
   	 	solved = true;
   	 	OMPL_INFORM("%s: Found Solution in %0.3f seconds!", getName().c_str(), solveTime_);
   	 	auto sol_plan = solution->getPlan();
   	 	for (int i = 0; i < sol_plan.size(); i++)
   	 	{
   	 	   auto path(std::make_shared<PathControl>(sol_plan[i]));
   	 	   mrmp_pdef_->getRobotProblemDefinitionPtr(i)->addSolutionPath(path, false, -1.0, getName().c_str());
   	 	}
   	 	OMPL_INFORM("%s: Planning Complete.", getName().c_str());
   	 	return {solved, false};
   	}
}
