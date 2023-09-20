#include "utils/OmplSetUp.h"

// this function sets-up an ompl planning problem for an arbtrary number of agents
// returns planners to be used
std::vector<MotionPlanningProblemPtr> set_up_all_MP_Problems(InstancePtr mrmp_instance)
{
    OMPL_INFORM("%s: Setting up planning problem for all agents.", "OMPL Set-Up");
    std::string mrmp_solver = mrmp_instance->getPlannerName();
    std::string ll_solver = mrmp_instance->getLowLevelPlannerName();
    if (mrmp_solver == "K-CBS" || mrmp_solver == "PBS") {
        if (ll_solver == "RRT") {
            return set_up_ConstraintRRT_MP_Problems(mrmp_instance);
        }
        else if (ll_solver == "BSST") {
            return set_up_ConstraintBSST_MP_Problems(mrmp_instance);
        }
    }
    else if (mrmp_solver == "MR-RRT") {
        return set_up_MultiRobotRRT_MP_Problem(mrmp_instance);
    }
    else if (mrmp_solver == "CentralizedBSST") {
        return set_up_CentralizedBSST_Problem(mrmp_instance);
    }
    else {
        OMPL_ERROR("%s: %s is not yet implemented for MRMP.", "OMPL Set-Up", mrmp_solver.c_str());
    }
    return {};
}

std::vector<MotionPlanningProblemPtr> set_up_ConstraintRRT_MP_Problems(InstancePtr mrmp_instance)
{
    const double goalRadius = 0.25;
    const double pi = bm::constants::pi<double>();
    const double stepSize = 0.1;
    
    std::vector<MotionPlanningProblemPtr> prob_defs{};

    std::vector<Robot*> robots= mrmp_instance->getRobots();
    for (auto itr = robots.begin(); itr != robots.end(); itr++) {
        if ((*itr)->getDynamicsModel() == "FirstOrderCar") {
            // set-up SE2 State Space
            auto space(std::make_shared<ob::SE2StateSpace>());
            ob::RealVectorBounds bounds(2);
            bounds.setLow(0, -1);
            bounds.setHigh(0, mrmp_instance->getDimensions()[0]);
            bounds.setLow(1, -1);
            bounds.setHigh(1, mrmp_instance->getDimensions()[1]);
            space->setBounds(bounds);
            

            // set-up the real vector control space 
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
            ob::RealVectorBounds cbounds(2);
            cbounds.setLow(0, -1);
            cbounds.setHigh(0, 1);
            cbounds.setLow(1, -pi/2);
            cbounds.setHigh(1, pi/2);
            cspace->setBounds(cbounds);

            // construct an instance of space information from this state/control space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            // construct (and include) an instance of PCCBlackmore State Validity Checker
            si->setStateValidityChecker(std::make_shared<RealVectorStateSpaceSVC>(si, mrmp_instance, (*itr)));
    
            // construct (and include) an instance of 2D-Uncertain-Linear State Propogator
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &FirstOrderCarODE));
            si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 
                &FirstOrderCarODEPostIntegration));
            // assume that planner integrates dynamics at steps of 0.1 seconds
            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);
            si->setup();

            ob::ScopedState<ob::SE2StateSpace> start(space);
            start->setX((*itr)->getStartLocation().x_);
            start->setY((*itr)->getStartLocation().y_);
            start->setYaw(0.0);  // start yaw is set manually (TODO)

            ob::ScopedState<> goal_loc(space);
            goal_loc[0] = ((*itr)->getGoalLocation().x_);
            goal_loc[1] = ((*itr)->getGoalLocation().y_);

            ob::GoalPtr goal (new R2Goal(si, goal_loc, goalRadius));
            
            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // // set the start and goal states
            // pdef->addStartState(start);
            // pdef->setGoal(goal);

            // // create (and provide) the low-level motion planner object
            // ConstraintValidityCheckerPtr validator = std::make_shared<DeterministicCVC>(*itr);
            // ConstraintRespectingPlannerPtr planner(std::make_shared<oc::ConstraintRespectingRRT>(si));
            // planner->as<oc::ConstraintRespectingRRT>()->setProblemDefinition(pdef);
            // planner->as<oc::ConstraintRespectingRRT>()->setConstraintValidator(validator);
            // planner->as<oc::ConstraintRespectingRRT>()->setup();

            // // append to MRMP problem list
            // auto mp = std::make_shared<MotionPlanningProblem>(si, pdef, planner);
            // prob_defs.push_back(mp);
        }
        else if ((*itr)->getDynamicsModel() == "SecondOrderCar") {
            // set-up the compound State Space
            auto space(std::make_shared<ob::CompoundStateSpace>());
            space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
            space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
            ob::RealVectorBounds bounds(4);
            bounds.setLow(0, -1); //  x lower bound
            bounds.setHigh(0, mrmp_instance->getDimensions()[0]); // x upper bound
            bounds.setLow(1, -1);  // y lower bound
            bounds.setHigh(1, mrmp_instance->getDimensions()[1]); // y upper bound
            bounds.setLow(2, -1);  // v lower bound
            bounds.setHigh(2, 1); // v upper bound
            bounds.setLow(3, -pi/4);  // phi lower bound
            bounds.setHigh(3, pi/4); // phi upper bound
            space->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);

            // set-up the real vector control space 
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
            ob::RealVectorBounds cbounds(2);
            cbounds.setLow(-1);
            cbounds.setHigh(1);
            cspace->setBounds(cbounds);

            // construct an instance of space information from this state/control space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            // construct (and include) an instance of PCCBlackmore State Validity Checker
            si->setStateValidityChecker(std::make_shared<RealVectorStateSpaceSVC>(si, mrmp_instance, (*itr)));
    
            // construct (and include) an instance of 2D-Uncertain-Linear State Propogator
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &SecondOrderCarODE));
            si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 
                &SecondOrderCarODEPostIntegration));
            // assume that planner integrates dynamics at steps of 0.1 seconds
            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);
            si->setup();

            ob::ScopedState<> start(space);
            start[0] = (*itr)->getStartLocation().x_;
            start[1] = (*itr)->getStartLocation().y_;
            start[2] = 0.0; // start v is set manually (TODO)
            start[3] = 0.0; // start phi is set manually (TODO)
            start[4] = 0.0;  // start theta is set manually (TODO)

            ob::ScopedState<> goal_loc(space);
            goal_loc[0] = ((*itr)->getGoalLocation().x_);
            goal_loc[1] = ((*itr)->getGoalLocation().y_);

            ob::GoalPtr goal (new R2Goal(si, goal_loc, goalRadius));
            
            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal states
            pdef->addStartState(start);
            pdef->setGoal(goal);

            // create (and provide) the low-level motion planner object
            // ConstraintValidityCheckerPtr validator = std::make_shared<DeterministicCVC>(*itr);
            // ConstraintRespectingPlannerPtr planner(std::make_shared<oc::ConstraintRespectingRRT>(si));
            // planner->as<oc::ConstraintRespectingRRT>()->setProblemDefinition(pdef);
            // planner->as<oc::ConstraintRespectingRRT>()->setConstraintValidator(validator);
            // planner->as<oc::ConstraintRespectingRRT>()->setup();

            // // append to MRMP problem list
            // auto mp = std::make_shared<MotionPlanningProblem>(si, pdef, planner);
            // prob_defs.push_back(mp);
        }
        else {
            OMPL_ERROR("%s: Dynamics model named %s is not yet implemented!", 
                "OMPL Set-Up", (*itr)->getDynamicsModel().c_str());
            return {};
        }
    }
    OMPL_INFORM("%s: Initialized %lu robots.", "OMPL Set-Up", prob_defs.size());
    return prob_defs;
}

std::vector<MotionPlanningProblemPtr> set_up_ConstraintBSST_MP_Problems(InstancePtr mrmp_instance)
{
    const double goalTollorance = 2.0;
    const double stepSize = 0.15; // 0.15
    
    std::vector<MotionPlanningProblemPtr> prob_defs{};

    std::vector<Robot*> robots= mrmp_instance->getRobots();
    for (auto itr = robots.begin(); itr != robots.end(); itr++) {
        if ((*itr)->getDynamicsModel() == "2D-Uncertain-Linear-Model") {
            // set-up 2D Belief Space
            ob::StateSpacePtr space = ob::StateSpacePtr(new RealVectorBeliefSpace(2));
            ob::RealVectorBounds bounds_se2(2);
            bounds_se2.setLow(0, -1);
            bounds_se2.setHigh(0, mrmp_instance->getDimensions()[0]);
            bounds_se2.setLow(1, -1);
            bounds_se2.setHigh(1, mrmp_instance->getDimensions()[1]);
            space->as<RealVectorBeliefSpace>()->setBounds(bounds_se2);

            // set-up the real vector control space
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
            ob::RealVectorBounds c_bounds(2);
            c_bounds.setLow(-1.0);  // this was [-100.0, 100.0]!
            c_bounds.setHigh(1.0);
            cspace->setBounds(c_bounds);

            // construct an instance of space information from this state/control space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            // construct (and include) an instance of PCCBlackmore State Validity Checker
            if (mrmp_instance->getSVC() == "Blackmore") {
                si->setStateValidityChecker(std::make_shared<PCCBlackmoreSVC>(si, mrmp_instance, (*itr), mrmp_instance->getPsafeObs()));
            }
            else if (mrmp_instance->getSVC() == "AdaptiveBlackmore") {
                si->setStateValidityChecker(std::make_shared<AdaptiveRiskBlackmoreSVC>(si, mrmp_instance, (*itr), mrmp_instance->getPsafeObs()));
            }
            else if (mrmp_instance->getSVC() == "ChiSquared") {
                // si->setStateValidityChecker(std::make_shared<ChiSquaredBoundarySVC>(si, mrmp_instance, (*itr), mrmp_instance->getPsafeObs()));
                si->setStateValidityChecker(std::make_shared<ChiSquaredBoundarySVC>(si, mrmp_instance, (*itr), 0.9));
            }

            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);
            
            // construct (and include) an instance of 2D-Uncertain-Linear State Propogator
            si->setStatePropagator(oc::StatePropagatorPtr(new R2_UncertainLinearStatePropagator(si)));

            si->setup();

            ob::State *start = si->allocState();
            start->as<RealVectorBeliefSpace::StateType>()->values[0] = (*itr)->getStartLocation().x_;
            start->as<RealVectorBeliefSpace::StateType>()->values[1] = (*itr)->getStartLocation().y_;
            Eigen::MatrixXd Sigma0 = 0.01 * Eigen::MatrixXd::Identity(2, 2);
            start->as<RealVectorBeliefSpace::StateType>()->sigma_ = Sigma0;

            // // create goal
            ob::GoalPtr goal(new ChanceConstrainedGoal(si, (*itr)->getGoalLocation(), goalTollorance, 0.95));

            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal states
            pdef->addStartState(start);
            pdef->setGoal(goal);

            // set optimization objective
            pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));

            // // create (and provide) the low-level motion planner object
            PlannerPtr planner(std::make_shared<oc::ConstraintRespectingBSST>(si));
            // ConstraintRespectingPlannerPtr planner(std::make_shared<oc::ConstraintRespectingBSST>(si));
            planner->as<oc::ConstraintRespectingBSST>()->setProblemDefinition(pdef);
            planner->as<oc::ConstraintRespectingBSST>()->setup();

            // append to MRMP problem list
            auto mp = std::make_shared<MotionPlanningProblem>(si, pdef, planner);
            prob_defs.push_back(mp);
        }
        else if ((*itr)->getDynamicsModel() == "Uncertain-Unicycle-Model") {
            // set-up Belief Space
            ob::StateSpacePtr space = ob::StateSpacePtr(new RealVectorBeliefSpace(4));
            // ob::RealVectorBounds bounds(4);
            // bounds.setLow(0, -1); // x low
            // bounds.setHigh(0, mrmp_instance->getDimensions()[0]); // x high
            // bounds.setLow(1, -1); // y low
            // bounds.setHigh(1, mrmp_instance->getDimensions()[1]); // y high
            // bounds.setLow(2, -1); // xdot
            // bounds.setHigh(2, 1); // xdot
            // bounds.setLow(3, -1); // ydot
            // bounds.setHigh(3, 1); // ydot
            // space->as<RealVectorBeliefSpace>()->setBounds(bounds);

            ob::RealVectorBounds cbounds(4);
            cbounds.setLow(0, -1); // x low
            cbounds.setHigh(0, mrmp_instance->getDimensions()[0]); // x high
            cbounds.setLow(1, -1); // y low
            cbounds.setHigh(1, mrmp_instance->getDimensions()[1]); // y high
            cbounds.setLow(2, -M_PI); // yaw low
            cbounds.setHigh(2, M_PI); // yaw high
            cbounds.setLow(3, 0.01); // surge low
            cbounds.setHigh(3, 10.0); // surge high

            // set-up the real vector control space
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 4));
            cspace->setBounds(cbounds); // cspace bounds are identical to state space bounds

            space->as<RealVectorBeliefSpace>()->setBounds(cbounds);

            // construct an instance of space information from this state/control space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            // construct (and include) an instance of PCCBlackmore State Validity Checker
            if (mrmp_instance->getSVC() == "Blackmore") {
                si->setStateValidityChecker(std::make_shared<PCCBlackmoreSVC>(si, mrmp_instance, (*itr), mrmp_instance->getPsafeObs()));
            }
            else if (mrmp_instance->getSVC() == "AdaptiveBlackmore") {
                si->setStateValidityChecker(std::make_shared<AdaptiveRiskBlackmoreSVC>(si, mrmp_instance, (*itr), mrmp_instance->getPsafeObs()));
            }
            else if (mrmp_instance->getSVC() == "ChiSquared") {
                // si->setStateValidityChecker(std::make_shared<ChiSquaredBoundarySVC>(si, mrmp_instance, (*itr), mrmp_instance->getPsafeObs()));
                si->setStateValidityChecker(std::make_shared<ChiSquaredBoundarySVC>(si, mrmp_instance, (*itr), 0.9));
            }

            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);
            
            // construct (and include) an instance of 2D-Uncertain-Linear State Propogator
            si->setStatePropagator(oc::StatePropagatorPtr(new DynUnicycleControlSpace(si)));

            si->setup();

            ob::State *start = si->allocState();
            start->as<RealVectorBeliefSpace::StateType>()->values[0] = (*itr)->getStartLocation().x_;
            start->as<RealVectorBeliefSpace::StateType>()->values[1] = (*itr)->getStartLocation().y_;
            start->as<RealVectorBeliefSpace::StateType>()->values[2] = 0; // initial yaw
            start->as<RealVectorBeliefSpace::StateType>()->values[3] = 0.1; // initial surge
            Eigen::MatrixXd Sigma0 = 0.01 * Eigen::MatrixXd::Identity(4, 4);
            start->as<RealVectorBeliefSpace::StateType>()->sigma_ = Sigma0;

            // // create goal
            ob::GoalPtr goal(new ChanceConstrainedGoal(si, (*itr)->getGoalLocation(), goalTollorance, 0.95));

            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal states
            pdef->addStartState(start);
            pdef->setGoal(goal);

            // set optimization objective
            pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));

            // create (and provide) the low-level motion planner object
            PlannerPtr planner(std::make_shared<oc::ConstraintRespectingBSST>(si));
            planner->as<oc::ConstraintRespectingBSST>()->setProblemDefinition(pdef);
            planner->as<oc::ConstraintRespectingBSST>()->setup();

            // append to MRMP problem list
            auto mp = std::make_shared<MotionPlanningProblem>(si, pdef, planner);
            prob_defs.push_back(mp);
        }
        else {
            OMPL_ERROR("%s: Dynamics model named %s is not yet implemented!", 
                "OMPL Set-Up", (*itr)->getDynamicsModel().c_str());
            return {};
        }
    }
    OMPL_INFORM("%s: Initialized %lu robots.", "OMPL Set-Up", prob_defs.size());
    return prob_defs;
}

std::vector<MotionPlanningProblemPtr> set_up_CentralizedBSST_Problem(InstancePtr mrmp_instance)
{
    const double goalTollorance = 2.0;
    const double stepSize = 0.5;

    std::vector<MotionPlanningProblemPtr> prob_defs{};

    if (mrmp_instance->getRobots().size() == 2) {
        Robot* r1 = mrmp_instance->getRobots()[0];
        Robot* r2 = mrmp_instance->getRobots()[1];
        if (r1->getDynamicsModel() == "2D-Uncertain-Linear-Model" && r2->getDynamicsModel() == "2D-Uncertain-Linear-Model") {
            // set-up 4D Belief Space
            ob::StateSpacePtr space = ob::StateSpacePtr(new RealVectorBeliefSpace(4));
            ob::RealVectorBounds bounds(4);
            bounds.setLow(0, -1);
            bounds.setHigh(0, mrmp_instance->getDimensions()[0]);
            bounds.setLow(1, -1);
            bounds.setHigh(1, mrmp_instance->getDimensions()[1]);
            bounds.setLow(2, -1);
            bounds.setHigh(2, mrmp_instance->getDimensions()[0]);
            bounds.setLow(3, -1);
            bounds.setHigh(3, mrmp_instance->getDimensions()[1]);
            space->as<RealVectorBeliefSpace>()->setBounds(bounds);

            // set-up the real vector control space
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 4));
            ob::RealVectorBounds c_bounds(4);
            c_bounds.setLow(-1.0);  // this was [-100.0, 100.0]!
            c_bounds.setHigh(1.0);
            cspace->setBounds(c_bounds);

            // construct an instance of space information from this state/control space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);

            // set State Propogator
            si->setStatePropagator(oc::StatePropagatorPtr(new CentralizedUncertainLinearStatePropagator(si)));

            // set State Validity Checker
            si->setStateValidityChecker(std::make_shared<CentralizedChiSquaredBoundarySVC>(si, mrmp_instance, mrmp_instance->getPsafe()));

            si->setup();

            // create start state
            ob::State *start = si->allocState();
            start->as<RealVectorStateSpace::StateType>()->values[0] = r1->getStartLocation().x_;
            start->as<RealVectorStateSpace::StateType>()->values[1] = r1->getStartLocation().y_;
            start->as<RealVectorStateSpace::StateType>()->values[2] = r2->getStartLocation().x_;
            start->as<RealVectorStateSpace::StateType>()->values[3] = r2->getStartLocation().y_;
            Eigen::MatrixXd Sigma0 = 0.00001 * Eigen::MatrixXd::Identity(4, 4);
            start->as<RealVectorBeliefSpace::StateType>()->sigma_ = Sigma0;

            // create goal state
            ob::State *goal_st = si->allocState();
            goal_st->as<RealVectorStateSpace::StateType>()->values[0] = r1->getGoalLocation().x_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[1] = r1->getGoalLocation().y_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[2] = r2->getGoalLocation().x_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[3] = r2->getGoalLocation().y_;

            // create goal object
            ob::GoalPtr goal(new CentralizedCCGoal(si, goal_st, goalTollorance, 0.5));

            // create ProblemDefinition
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal
            pdef->addStartState(start);
            pdef->setGoal(goal);

            // set optimization objective
            pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));

            // create (and provide) the motion planner object
            PlannerPtr planner(std::make_shared<oc::CentralizedBSST>(si, 2));
            planner->as<oc::CentralizedBSST>()->setProblemDefinition(pdef);
            planner->as<oc::CentralizedBSST>()->setup();
            // append to MRMP problem list
            auto mp = std::make_shared<MotionPlanningProblem>(si, pdef, planner);
            prob_defs.push_back(mp);
        }
        else {
            OMPL_ERROR("Provided Dynamics combination not yet implemented for centralized planning!");
            exit(-1);
        }
    }
    else if (mrmp_instance->getRobots().size() == 3) {
        Robot* r1 = mrmp_instance->getRobots()[0];
        Robot* r2 = mrmp_instance->getRobots()[1];
        Robot* r3 = mrmp_instance->getRobots()[2];
        if (r1->getDynamicsModel() == "2D-Uncertain-Linear-Model" && 
                r2->getDynamicsModel() == "2D-Uncertain-Linear-Model" && 
                r3->getDynamicsModel() == "2D-Uncertain-Linear-Model") {
            // set-up 6D Belief Space
            ob::StateSpacePtr space = ob::StateSpacePtr(new RealVectorBeliefSpace(6));
            ob::RealVectorBounds bounds(6);
            bounds.setLow(0, -1);
            bounds.setHigh(0, mrmp_instance->getDimensions()[0]);
            bounds.setLow(1, -1);
            bounds.setHigh(1, mrmp_instance->getDimensions()[1]);
            bounds.setLow(2, -1);
            bounds.setHigh(2, mrmp_instance->getDimensions()[0]);
            bounds.setLow(3, -1);
            bounds.setHigh(3, mrmp_instance->getDimensions()[1]);
            bounds.setLow(4, -1);
            bounds.setHigh(4, mrmp_instance->getDimensions()[0]);
            bounds.setLow(5, -1);
            bounds.setHigh(5, mrmp_instance->getDimensions()[1]);

            space->as<RealVectorBeliefSpace>()->setBounds(bounds);

            // set-up the real vector control space
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 6));
            ob::RealVectorBounds c_bounds(6);
            c_bounds.setLow(-1.0);  // this was [-100.0, 100.0]!
            c_bounds.setHigh(1.0);
            cspace->setBounds(c_bounds);

            // construct an instance of space information from this state/control space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);

            // set State Propogator
            si->setStatePropagator(oc::StatePropagatorPtr(new CentralizedUncertainLinearStatePropagator(si)));

            // set State Validity Checker
            si->setStateValidityChecker(std::make_shared<CentralizedChiSquaredBoundarySVC>(si, mrmp_instance, mrmp_instance->getPsafe()));

            si->setup();

            // create start state
            ob::State *start = si->allocState();
            start->as<RealVectorStateSpace::StateType>()->values[0] = r1->getStartLocation().x_;
            start->as<RealVectorStateSpace::StateType>()->values[1] = r1->getStartLocation().y_;
            start->as<RealVectorStateSpace::StateType>()->values[2] = r2->getStartLocation().x_;
            start->as<RealVectorStateSpace::StateType>()->values[3] = r2->getStartLocation().y_;
            start->as<RealVectorStateSpace::StateType>()->values[4] = r3->getStartLocation().x_;
            start->as<RealVectorStateSpace::StateType>()->values[5] = r3->getStartLocation().y_;
            Eigen::MatrixXd Sigma0 = 0.00001 * Eigen::MatrixXd::Identity(6, 6);
            start->as<RealVectorBeliefSpace::StateType>()->sigma_ = Sigma0;

            // create goal state
            ob::State *goal_st = si->allocState();
            goal_st->as<RealVectorStateSpace::StateType>()->values[0] = r1->getGoalLocation().x_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[1] = r1->getGoalLocation().y_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[2] = r2->getGoalLocation().x_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[3] = r2->getGoalLocation().y_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[4] = r3->getGoalLocation().x_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[5] = r3->getGoalLocation().y_;

            // create goal object
            ob::GoalPtr goal(new CentralizedCCGoal(si, goal_st, goalTollorance, 0.95));

            // create ProblemDefinition
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal
            pdef->addStartState(start);
            pdef->setGoal(goal);

            // set optimization objective
            pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));

            // create (and provide) the motion planner object
            PlannerPtr planner(std::make_shared<oc::CentralizedBSST>(si, 3));
            planner->as<oc::CentralizedBSST>()->setProblemDefinition(pdef);
            planner->as<oc::CentralizedBSST>()->setup();
            // append to MRMP problem list
            auto mp = std::make_shared<MotionPlanningProblem>(si, pdef, planner);
            prob_defs.push_back(mp);
        }
        else {
            OMPL_ERROR("Provided Dynamics combination not yet implemented for centralized planning!");
            exit(-1);
        }
    }
    else if (mrmp_instance->getRobots().size() == 4) {
        Robot* r1 = mrmp_instance->getRobots()[0];
        Robot* r2 = mrmp_instance->getRobots()[1];
        Robot* r3 = mrmp_instance->getRobots()[2];
        Robot* r4 = mrmp_instance->getRobots()[3];
        if (r1->getDynamicsModel() == "2D-Uncertain-Linear-Model" && 
                r2->getDynamicsModel() == "2D-Uncertain-Linear-Model" && 
                r3->getDynamicsModel() == "2D-Uncertain-Linear-Model" &&
                r4->getDynamicsModel() == "2D-Uncertain-Linear-Model") {
            // set-up 6D Belief Space
            ob::StateSpacePtr space = ob::StateSpacePtr(new RealVectorBeliefSpace(8));
            ob::RealVectorBounds bounds(8);
            bounds.setLow(0, -1);
            bounds.setHigh(0, mrmp_instance->getDimensions()[0]);
            bounds.setLow(1, -1);
            bounds.setHigh(1, mrmp_instance->getDimensions()[1]);
            bounds.setLow(2, -1);
            bounds.setHigh(2, mrmp_instance->getDimensions()[0]);
            bounds.setLow(3, -1);
            bounds.setHigh(3, mrmp_instance->getDimensions()[1]);
            bounds.setLow(4, -1);
            bounds.setHigh(4, mrmp_instance->getDimensions()[0]);
            bounds.setLow(5, -1);
            bounds.setHigh(5, mrmp_instance->getDimensions()[1]);
            bounds.setLow(6, -1);
            bounds.setHigh(6, mrmp_instance->getDimensions()[0]);
            bounds.setLow(7, -1);
            bounds.setHigh(7, mrmp_instance->getDimensions()[1]);

            space->as<RealVectorBeliefSpace>()->setBounds(bounds);

            // set-up the real vector control space
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 8));
            ob::RealVectorBounds c_bounds(8);
            c_bounds.setLow(-1.0);  // this was [-100.0, 100.0]!
            c_bounds.setHigh(1.0);
            cspace->setBounds(c_bounds);

            // construct an instance of space information from this state/control space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);

            // set State Propogator
            si->setStatePropagator(oc::StatePropagatorPtr(new CentralizedUncertainLinearStatePropagator(si)));

            // set State Validity Checker
            si->setStateValidityChecker(std::make_shared<CentralizedChiSquaredBoundarySVC>(si, mrmp_instance, mrmp_instance->getPsafe()));

            si->setup();

            // create start state
            ob::State *start = si->allocState();
            start->as<RealVectorStateSpace::StateType>()->values[0] = r1->getStartLocation().x_;
            start->as<RealVectorStateSpace::StateType>()->values[1] = r1->getStartLocation().y_;
            start->as<RealVectorStateSpace::StateType>()->values[2] = r2->getStartLocation().x_;
            start->as<RealVectorStateSpace::StateType>()->values[3] = r2->getStartLocation().y_;
            start->as<RealVectorStateSpace::StateType>()->values[4] = r3->getStartLocation().x_;
            start->as<RealVectorStateSpace::StateType>()->values[5] = r3->getStartLocation().y_;
            start->as<RealVectorStateSpace::StateType>()->values[6] = r4->getStartLocation().x_;
            start->as<RealVectorStateSpace::StateType>()->values[7] = r4->getStartLocation().y_;
            Eigen::MatrixXd Sigma0 = 0.00001 * Eigen::MatrixXd::Identity(8, 8);
            start->as<RealVectorBeliefSpace::StateType>()->sigma_ = Sigma0;

            // create goal state
            ob::State *goal_st = si->allocState();
            goal_st->as<RealVectorStateSpace::StateType>()->values[0] = r1->getGoalLocation().x_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[1] = r1->getGoalLocation().y_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[2] = r2->getGoalLocation().x_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[3] = r2->getGoalLocation().y_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[4] = r3->getGoalLocation().x_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[5] = r3->getGoalLocation().y_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[6] = r4->getGoalLocation().x_;
            goal_st->as<RealVectorStateSpace::StateType>()->values[7] = r4->getGoalLocation().y_;

            // create goal object
            ob::GoalPtr goal(new CentralizedCCGoal(si, goal_st, goalTollorance, 0.95));

            // create ProblemDefinition
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal
            pdef->addStartState(start);
            pdef->setGoal(goal);

            // set optimization objective
            pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));

            // create (and provide) the motion planner object
            PlannerPtr planner(std::make_shared<oc::CentralizedBSST>(si, 4));
            planner->as<oc::CentralizedBSST>()->setProblemDefinition(pdef);
            planner->as<oc::CentralizedBSST>()->setup();
            // append to MRMP problem list
            auto mp = std::make_shared<MotionPlanningProblem>(si, pdef, planner);
            prob_defs.push_back(mp);
        }
    }
    else {
        OMPL_ERROR("Unable to solve centralized instances with >3 robots!");
        exit(-1);
    }
    return prob_defs;
}


std::vector<MotionPlanningProblemPtr> set_up_MultiRobotRRT_MP_Problem(InstancePtr mrmp_instance)
{
        // for (auto itr = robots.begin(); itr != robots.end(); itr++) {
        
    // }
    //     else if (a->getDynamics() == "Two Dynamic Cars")
    //     {
    //         // Dynamic car operates in a compound state space
    //         auto space(std::make_shared<ob::CompoundStateSpace>());
    //         // real vector space for x1 y1 v1 phi1
    //         space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
    //         // next, add a SO2StateSpace for theta1
    //         space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
    //         // real vector space for x2 y2 v2 phi2
    //         space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
    //         // next, add a SO2StateSpace for theta2
    //         space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

    //         // add bounds of space
    //         ob::RealVectorBounds bounds(4);
    //         bounds.setLow(0, 0); //  x lower bound
    //         bounds.setHigh(0, w->getWorldDimensions()[0]); // x upper bound
    //         bounds.setLow(1, 0);  // y lower bound
    //         bounds.setHigh(1, w->getWorldDimensions()[1]); // y upper bound
    //         bounds.setLow(2, -1);  // v lower bound
    //         bounds.setHigh(2, 1); // v upper bound
    //         bounds.setLow(3, -pi/4);  // phi lower bound
    //         bounds.setHigh(3, pi/4); // phi upper bound

    //         // set the bounds of this space 
    //         space->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    //         space->as<ob::RealVectorStateSpace>(2)->setBounds(bounds);

    //         // Create control space and set bounds
    //         auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 4));
    //         ob::RealVectorBounds cbounds(4);
    //         cbounds.setLow(-1);
    //         cbounds.setHigh(1);

    //         cspace->setBounds(cbounds);

    //         // construct an instance of  space information from this state space
    //         auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

    //         si->setStateValidityChecker(
    //             std::make_shared<isComposedStateValid_2D>(si, w, a, a->getDynamics()));

    //         // Use the ODESolver to propagate the system.  
    //         auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &TwoDynamicCarsODE));
    //         si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 
    //             &TwoDynamicCarsPostIntegration));
    //         // assume that planner integrates dynamics at steps of 0.1 seconds
    //         si->setPropagationStepSize(stepSize);
    //         si->setMinMaxControlDuration(1, 10);
    //         si->setup();

    //         // create start
    //         ob::ScopedState<> start(space);
    //         start[0] = a->getStartLocation()[0];
    //         start[1] = a->getStartLocation()[1];
    //         start[2] = 0.0; // start v is set manually (TODO)
    //         start[3] = 0.0; // start phi is set manually (TODO)
    //         start[4] = 0.0;  // start theta is set manually (TODO)
    //         start[5] = a->getStartLocation()[2];
    //         start[6] = a->getStartLocation()[3];
    //         start[7] = 0.0; // start v is set manually (TODO)
    //         start[8] = 0.0; // start phi is set manually (TODO)
    //         start[9] = 0.0;  // start theta is set manually (TODO)

    //         // create goal
    //         ob::GoalPtr goal (new ArbirtryComposedGoal_2D(si, 
    //             a->getGoalLocation(), goalRadius, a->getDynamics()));

    //         // create a problem instance
    //         auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    //         // set the start and goal
    //         pdef->addStartState(start);
    //         pdef->setGoal(goal);

    //         problem prob(si, pdef);

    //         probDefs.push_back(prob);
    //     }
    //     else if (a->getDynamics() == "Three Dynamic Cars")
    //     {
    //         // Dynamic car operates in a compound state space
    //         auto space(std::make_shared<ob::CompoundStateSpace>());
    //         // real vector space for x1 y1 v1 phi1
    //         space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
    //         // next, add a SO2StateSpace for theta1
    //         space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
    //         // real vector space for x2 y2 v2 phi2
    //         space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
    //         // next, add a SO2StateSpace for theta2
    //         space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
    //         // real vector space for x3 y3 v3 phi3
    //         space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
    //         // next, add a SO2StateSpace for theta3
    //         space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

    //         // add bounds of space
    //         ob::RealVectorBounds bounds(4);
    //         bounds.setLow(0, 0); //  x lower bound
    //         bounds.setHigh(0, w->getWorldDimensions()[0]); // x upper bound
    //         bounds.setLow(1, 0);  // y lower bound
    //         bounds.setHigh(1, w->getWorldDimensions()[1]); // y upper bound
    //         bounds.setLow(2, -1);  // v lower bound
    //         bounds.setHigh(2, 1); // v upper bound
    //         bounds.setLow(3, -pi/2);  // phi lower bound
    //         bounds.setHigh(3, pi/2); // phi upper bound

    //         // set the bounds of this space 
    //         space->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    //         space->as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
    //         space->as<ob::RealVectorStateSpace>(4)->setBounds(bounds);

    //         // Create control space and set bounds
    //         auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 6));
    //         ob::RealVectorBounds cbounds(6);
    //         cbounds.setLow(-1);
    //         cbounds.setHigh(1);

    //         cspace->setBounds(cbounds);

    //         // construct an instance of  space information from this state space
    //         auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

    //         si->setStateValidityChecker(
    //             std::make_shared<isComposedStateValid_2D>(si, w, a, a->getDynamics()));

    //         // Use the ODESolver to propagate the system.  
    //         auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &ThreeDynamicCarsODE));
    //         si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 
    //             &ThreeDynamicCarsPostIntegration));
    //         // assume that planner integrates dynamics at steps of 0.1 seconds
    //         si->setPropagationStepSize(stepSize);
    //         si->setMinMaxControlDuration(1, 10);
    //         si->setup();

    //         // create start
    //         ob::ScopedState<> start(space);
    //         start[0] = a->getStartLocation()[0];
    //         start[1] = a->getStartLocation()[1];
    //         start[2] = 0.0; // start v is set manually (TODO)
    //         start[3] = 0.0; // start phi is set manually (TODO)
    //         start[4] = 0.0;  // start theta is set manually (TODO)
    //         start[5] = a->getStartLocation()[2];
    //         start[6] = a->getStartLocation()[3];
    //         start[7] = 0.0; // start v is set manually (TODO)
    //         start[8] = 0.0; // start phi is set manually (TODO)
    //         start[9] = 0.0;  // start theta is set manually (TODO)
    //         start[10] = a->getStartLocation()[4];
    //         start[11] = a->getStartLocation()[5];
    //         start[12] = 0.0; // start v is set manually (TODO)
    //         start[13] = 0.0; // start phi is set manually (TODO)
    //         start[14] = 0.0;  // start theta is set manually (TODO)

    //         // create goal
    //         ob::GoalPtr goal (new ArbirtryComposedGoal_2D(si, 
    //             a->getGoalLocation(), goalRadius, a->getDynamics()));

    //         // create a problem instance
    //         auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    //         // set the start and goal
    //         pdef->addStartState(start);
    //         pdef->setGoal(goal);

    //         problem prob(si, pdef);

    //         probDefs.push_back(prob);
    //     }
    //     else
    //     {
    //         OMPL_ERROR("No model implemented for %s dynamics: %s", a->getName().c_str(), a->getDynamics().c_str());
    //         return probDefs;
    //     }
    OMPL_ERROR("%s: this is not yet implemented after code refactor.", "OMPL MultiRobotRRT Set-Up");
    return {};
}





