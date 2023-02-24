#include "utils/beliefCollisionCheckingBenchmark.h"

BeliefCollisionCheckerBenchmark::BeliefCollisionCheckerBenchmark(std::string file1, std::string file2):
    beliefList1_(file1), beliefList2_(file2)
{
    ob::StateSpacePtr space = ob::StateSpacePtr(new RealVectorBeliefSpace(2));
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // construct an instance of space information from this state/control space
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

    // create pdef
    auto pdef(std::make_shared<ob::ProblemDefinition>(si_));

    // create planner
    ConstraintRespectingPlannerPtr planner(std::make_shared<oc::ConstraintRespectingBSST>(si_));

    // create mp problem def
    auto mp_problem = std::make_shared<MotionPlanningProblem>(si_, pdef, planner);

    std::vector<MotionPlanningProblemPtr> mp_problems{mp_problem};
    mrmp_pdef_ = std::make_shared<MultiRobotProblemDefinition>(mp_problems);

    // create instance
    Location dummy_location(0, 0);
    Robot* r0 = new RectangularRobot("Robot 0", "N/A", dummy_location, dummy_location, 1, 1);
    Robot* r1 = new RectangularRobot("Robot 1", "N/A", dummy_location, dummy_location, 1, 1);
    instance_ = std::make_shared<Instance>(2, 0.95);

    instance_->addRobot(r0);
    instance_->addRobot(r1);

    // set instance
    mrmp_pdef_->setMultiRobotInstance(instance_);


    // fill the agent belief maps with belifList files
    std::ifstream myfile(beliefList1_);
    if (!myfile.is_open()) {
        OMPL_ERROR("Unable to open %s", beliefList1_);
        return;
    }
    
    std::string line;
    boost::tokenizer< boost::char_separator<char> >::iterator beg;
    boost::char_separator<char> sep(",");
    int idx = 0;
    while (getline(myfile, line)) {
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        beg = tok.begin();

        // create start state
        ob::State *st = si_->allocState();

        st->as<RealVectorStateSpace::StateType>()->values[0] = stod(*beg);;
        beg++;
        st->as<RealVectorStateSpace::StateType>()->values[1] = stod(*beg);
        beg++;
        st->as<RealVectorBeliefSpace::StateType>()->sigma_(0, 0) = stod(*beg);
        beg++;
        st->as<RealVectorBeliefSpace::StateType>()->sigma_(0, 1) = stod(*beg);
        beg++;
        st->as<RealVectorBeliefSpace::StateType>()->sigma_(1, 0) = stod(*beg);
        beg++;
        st->as<RealVectorBeliefSpace::StateType>()->sigma_(1, 1) = stod(*beg);

        agent1_belief_map_[idx] = st;
        idx++;
    }

    // fill the agent belief maps with belifList files
    std::ifstream myfile2(beliefList2_);
    if (!myfile2.is_open()) {
        OMPL_ERROR("Unable to open %s", beliefList2_);
        return;
    }
    
    idx = 0;
    while (getline(myfile2, line)) {
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        beg = tok.begin();

        // create start state
        ob::State *st = si_->allocState();

        st->as<RealVectorStateSpace::StateType>()->values[0] = stod(*beg);;
        beg++;
        st->as<RealVectorStateSpace::StateType>()->values[1] = stod(*beg);
        beg++;
        st->as<RealVectorBeliefSpace::StateType>()->sigma_(0, 0) = stod(*beg);
        beg++;
        st->as<RealVectorBeliefSpace::StateType>()->sigma_(0, 1) = stod(*beg);
        beg++;
        st->as<RealVectorBeliefSpace::StateType>()->sigma_(1, 0) = stod(*beg);
        beg++;
        st->as<RealVectorBeliefSpace::StateType>()->sigma_(1, 1) = stod(*beg);

        agent2_belief_map_[idx] = st;
        idx++;
    }
    assert(agent1_belief_map_.size() == agent2_belief_map_.size());
}


void BeliefCollisionCheckerBenchmark::runBenchmarks()
{
    /* Create results data structure */
    std::unordered_map<int, std::pair<double, bool>> chiSquared_results;
    const int num_examples = agent1_belief_map_.size();
    bool result;
    int num_fail = 0;

    /* Create state validity checker object */
    auto ChiSquaredPlanValidator = std::make_shared<ChiSquaredBoundaryPVC>(mrmp_pdef_, instance_->getPsafe());
    
    /* Iterate through all the states, test for collisions, and save results */
    for (int idx = 0; idx != num_examples; idx++) {
        auto start_time = std::chrono::system_clock::now();
        result = ChiSquaredPlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
        if (!result)
            num_fail++;
        std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
        double execution_time = elapsed_time.count();
        std::pair<double, bool> p(execution_time, result);
        chiSquared_results[idx] = p;
    }

    double stat = (double)num_fail / num_examples;
    std::cout << "Chi Squared: " << stat * 100.0 << std::endl;

    // exportResults("chi_squared_results.txt", chiSquared_results);

    /* Create results data structure */
    std::unordered_map<int, std::pair<double, bool>> sumBlackmore_results;
    num_fail = 0;

    /* Create state validity checker object */
    auto MinkowskiSumBlackmorePlanValidator = std::make_shared<MinkowskiSumBlackmorePVC>(mrmp_pdef_, instance_->getPsafe());
    
    /* Iterate through all the states, test for collisions, and save results */
    for (int idx = 0; idx != num_examples; idx++) {
        auto start_time = std::chrono::system_clock::now();
        result = MinkowskiSumBlackmorePlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
        if (!result)
            num_fail++;
        std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
        double execution_time = elapsed_time.count();
        std::pair<double, bool> p(execution_time, result);
        sumBlackmore_results[idx] = p;
    }

    stat = (double)num_fail / num_examples;
    std::cout << "MinkowskiSum: " << stat * 100.0 << std::endl;

    // exportResults("minkowski_sum_results.txt", sumBlackmore_results);

    /* Create results data structure */
    std::unordered_map<int, std::pair<double, bool>> boundingBox_results;
    num_fail = 0;

    /* Create state validity checker object */
    auto BoundingBoxBlackmorePlanValidator = std::make_shared<BoundingBoxBlackmorePVC>(mrmp_pdef_, instance_->getPsafe());
    
    /* Iterate through all the states, test for collisions, and save results */
    for (int idx = 0; idx != num_examples; idx++) {
        auto start_time = std::chrono::system_clock::now();
        result = BoundingBoxBlackmorePlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
        if (!result)
            num_fail++;
        std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
        double execution_time = elapsed_time.count();
        std::pair<double, bool> p(execution_time, result);
        boundingBox_results[idx] = p;
    }

    stat = (double)num_fail / num_examples;
    std::cout << "BoundingBox: " << stat * 100.0 << std::endl;

    /* Create results data structure */
    std::unordered_map<int, std::pair<double, bool>> adaptiveBlackmore_results;
    num_fail = 0;

    /* Create state validity checker object */
    auto AdaptiveBlackmorePlanValidator = std::make_shared<AdaptiveRiskBlackmorePVC>(mrmp_pdef_, instance_->getPsafe());
    
    /* Iterate through all the states, test for collisions, and save results */
    for (int idx = 0; idx != num_examples; idx++) {
        auto start_time = std::chrono::system_clock::now();
        result = AdaptiveBlackmorePlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
        if (!result)
            num_fail++;
        std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
        double execution_time = elapsed_time.count();
        std::pair<double, bool> p(execution_time, result);
        adaptiveBlackmore_results[idx] = p;
    }

    stat = (double)num_fail / num_examples;
    std::cout << "AdaptiveBlackmore: " << stat * 100.0 << std::endl;    
}

// void exportResults(std::string filename, std::unordered_map<int, std::pair<double, bool>> results)
// {


// }






