#include "includes/multiAgentSetUp.h"

// this function sets-up an ompl planning problem for an arbtrary number of agents
// returns planners to be used
const std::vector<problem> multiAgentSetUp(const World *w)
{
    std::cout << "" << std::endl;
    OMPL_INFORM("Setting up planning problem for all agents...");
    // std::vector<shared_ptr<oc::constraintRRT> >
    std::vector<problem> probDefs;
    const double goalRadius = 0.25;
    // define pi
    const double pi = boost::math::constants::pi<double>();
    const double stepSize = 0.08;
    for (int i=0; i < w->getAgents().size(); i++)
    {
        Agent *a = w->getAgents()[i];
        if (a->getDynamics() == "Kinematic Car")
        {
            // Kinematic Car operates in SE2
            auto space(std::make_shared<ob::SE2StateSpace>());

            // real vector bounds of space are unique to dimension / dynamics
            // get them from the World object
            ob::RealVectorBounds bounds(2);

            bounds.setLow(0, 0);
            bounds.setHigh(0, w->getWorldDimensions()[0]);
            bounds.setLow(1, 0);
            bounds.setHigh(1, w->getWorldDimensions()[1]);
 
            space->setBounds(bounds);

            // create a standard 2D control space 
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

            // set bounds
            ob::RealVectorBounds cbounds(2);
            cbounds.setLow(0, -1);
            cbounds.setHigh(0, 1);
            cbounds.setLow(1, -pi/2);
            cbounds.setHigh(1, pi/2);

            cspace->setBounds(cbounds);

            // construct an instance of  space information from this state space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            si->setStateValidityChecker(
                std::make_shared<isStateValid_2D>(si, w, a));
    
            // Use the ODESolver to propagate the system.  
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &KinematicCarODE));
            si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 
                &KinematicCarPostIntegration));
            // assume that planner integrates dynamics at steps of 0.1 seconds
            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);
            si->setup();

            ob::ScopedState<ob::SE2StateSpace> start(space);
            start->setX(a->getStartLocation()[0]);
            start->setY(a->getStartLocation()[1]);
            start->setYaw(0.0);  // start yaw is set manually (TODO)

            ob::GoalPtr goal (new ArbirtryGoal_2D(si, a->getGoalLocation(), goalRadius));
            
            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal states
            pdef->addStartState(start);
            pdef->setGoal(goal);

            problem prob(si, pdef);

            probDefs.push_back(prob);
        }
        else if (a->getDynamics() == "Dynamic Car")
        {
            // Dynamic car operates in a compound state space
            auto space(std::make_shared<ob::CompoundStateSpace>());
            // real vector space for x y v phi
            space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
            // next, add a SO2StateSpace for theta
            space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

            // add bounds of space
            ob::RealVectorBounds bounds(4);
            bounds.setLow(0, 0); //  x lower bound
            bounds.setHigh(0, w->getWorldDimensions()[0]); // x upper bound
            bounds.setLow(1, 0);  // y lower bound
            bounds.setHigh(1, w->getWorldDimensions()[1]); // y upper bound
            bounds.setLow(2, -1);  // v lower bound
            bounds.setHigh(2, 1); // v upper bound
            bounds.setLow(3, -pi/4);  // phi lower bound
            bounds.setHigh(3, pi/4); // phi upper bound

            // set bounds
            space->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);

            // Create control space and set bounds
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
            ob::RealVectorBounds cbounds(2);
            cbounds.setLow(-1);
            cbounds.setHigh(1);

            cspace->setBounds(cbounds);

            // construct an instance of  space information from this state space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            si->setStateValidityChecker(
                std::make_shared<isStateValid_2D>(si, w, a));

            // Use the ODESolver to propagate the system.  
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &DynamicCarODE));
            si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 
                &DynamicCarPostIntegration));
            // assume that planner integrates dynamics at steps of 0.1 seconds
            // si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);
            si->setup();

            // create start
            ob::ScopedState<> start(space);
            start[0] = a->getStartLocation()[0];
            start[1] = a->getStartLocation()[1];
            start[2] = 0.0; // start v is set manually (TODO)
            start[3] = 0.0; // start phi is set manually (TODO)
            start[4] = 0.0;  // start theta is set manually (TODO)

            // create goal
            ob::GoalPtr goal (new ArbirtryGoal_2D(si, a->getGoalLocation(), goalRadius));

            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal
            pdef->addStartState(start);
            pdef->setGoal(goal);

            problem prob(si, pdef);

            probDefs.push_back(prob);
        }
        else if (a->getDynamics() == "Two Dynamic Cars")
        {
            // Dynamic car operates in a compound state space
            auto space(std::make_shared<ob::CompoundStateSpace>());
            // real vector space for x1 y1 v1 phi1
            space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
            // next, add a SO2StateSpace for theta1
            space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
            // real vector space for x2 y2 v2 phi2
            space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
            // next, add a SO2StateSpace for theta2
            space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

            // add bounds of space
            ob::RealVectorBounds bounds(4);
            bounds.setLow(0, 0); //  x lower bound
            bounds.setHigh(0, w->getWorldDimensions()[0]); // x upper bound
            bounds.setLow(1, 0);  // y lower bound
            bounds.setHigh(1, w->getWorldDimensions()[1]); // y upper bound
            bounds.setLow(2, -1);  // v lower bound
            bounds.setHigh(2, 1); // v upper bound
            bounds.setLow(3, -pi/4);  // phi lower bound
            bounds.setHigh(3, pi/4); // phi upper bound

            // set the bounds of this space 
            space->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
            space->as<ob::RealVectorStateSpace>(2)->setBounds(bounds);

            // Create control space and set bounds
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 4));
            ob::RealVectorBounds cbounds(4);
            cbounds.setLow(-1);
            cbounds.setHigh(1);

            cspace->setBounds(cbounds);

            // construct an instance of  space information from this state space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            si->setStateValidityChecker(
                std::make_shared<isComposedStateValid_2D>(si, w, a, a->getDynamics()));

            // Use the ODESolver to propagate the system.  
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &TwoDynamicCarsODE));
            si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 
                &TwoDynamicCarsPostIntegration));
            // assume that planner integrates dynamics at steps of 0.1 seconds
            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);
            si->setup();

            // create start
            ob::ScopedState<> start(space);
            start[0] = a->getStartLocation()[0];
            start[1] = a->getStartLocation()[1];
            start[2] = 0.0; // start v is set manually (TODO)
            start[3] = 0.0; // start phi is set manually (TODO)
            start[4] = 0.0;  // start theta is set manually (TODO)
            start[5] = a->getStartLocation()[2];
            start[6] = a->getStartLocation()[3];
            start[7] = 0.0; // start v is set manually (TODO)
            start[8] = 0.0; // start phi is set manually (TODO)
            start[9] = 0.0;  // start theta is set manually (TODO)

            // create goal
            ob::GoalPtr goal (new ArbirtryComposedGoal_2D(si, 
                a->getGoalLocation(), goalRadius, a->getDynamics()));

            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal
            pdef->addStartState(start);
            pdef->setGoal(goal);

            problem prob(si, pdef);

            probDefs.push_back(prob);
        }
        else if (a->getDynamics() == "Three Dynamic Cars")
        {
            // Dynamic car operates in a compound state space
            auto space(std::make_shared<ob::CompoundStateSpace>());
            // real vector space for x1 y1 v1 phi1
            space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
            // next, add a SO2StateSpace for theta1
            space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
            // real vector space for x2 y2 v2 phi2
            space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
            // next, add a SO2StateSpace for theta2
            space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
            // real vector space for x3 y3 v3 phi3
            space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
            // next, add a SO2StateSpace for theta3
            space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);

            // add bounds of space
            ob::RealVectorBounds bounds(4);
            bounds.setLow(0, 0); //  x lower bound
            bounds.setHigh(0, w->getWorldDimensions()[0]); // x upper bound
            bounds.setLow(1, 0);  // y lower bound
            bounds.setHigh(1, w->getWorldDimensions()[1]); // y upper bound
            bounds.setLow(2, -1);  // v lower bound
            bounds.setHigh(2, 1); // v upper bound
            bounds.setLow(3, -pi/2);  // phi lower bound
            bounds.setHigh(3, pi/2); // phi upper bound

            // set the bounds of this space 
            space->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
            space->as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
            space->as<ob::RealVectorStateSpace>(4)->setBounds(bounds);

            // Create control space and set bounds
            auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 6));
            ob::RealVectorBounds cbounds(6);
            cbounds.setLow(-1);
            cbounds.setHigh(1);

            cspace->setBounds(cbounds);

            // construct an instance of  space information from this state space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            si->setStateValidityChecker(
                std::make_shared<isComposedStateValid_2D>(si, w, a, a->getDynamics()));

            // Use the ODESolver to propagate the system.  
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &ThreeDynamicCarsODE));
            si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 
                &ThreeDynamicCarsPostIntegration));
            // assume that planner integrates dynamics at steps of 0.1 seconds
            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 10);
            si->setup();

            // create start
            ob::ScopedState<> start(space);
            start[0] = a->getStartLocation()[0];
            start[1] = a->getStartLocation()[1];
            start[2] = 0.0; // start v is set manually (TODO)
            start[3] = 0.0; // start phi is set manually (TODO)
            start[4] = 0.0;  // start theta is set manually (TODO)
            start[5] = a->getStartLocation()[2];
            start[6] = a->getStartLocation()[3];
            start[7] = 0.0; // start v is set manually (TODO)
            start[8] = 0.0; // start phi is set manually (TODO)
            start[9] = 0.0;  // start theta is set manually (TODO)
            start[10] = a->getStartLocation()[4];
            start[11] = a->getStartLocation()[5];
            start[12] = 0.0; // start v is set manually (TODO)
            start[13] = 0.0; // start phi is set manually (TODO)
            start[14] = 0.0;  // start theta is set manually (TODO)

            // create goal
            ob::GoalPtr goal (new ArbirtryComposedGoal_2D(si, 
                a->getGoalLocation(), goalRadius, a->getDynamics()));

            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal
            pdef->addStartState(start);
            pdef->setGoal(goal);

            problem prob(si, pdef);

            probDefs.push_back(prob);
        }
        else
        {
            OMPL_ERROR("No model implemented for %s dynamics: %s", a->getName().c_str(), a->getDynamics().c_str());
            return probDefs;
        }
    }
    OMPL_INFORM("Done!");
    return probDefs;
}