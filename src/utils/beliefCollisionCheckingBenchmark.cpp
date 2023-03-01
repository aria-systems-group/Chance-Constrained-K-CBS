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

        agent1_belief_map_.push_back(st);
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

        agent2_belief_map_.push_back(st);
        idx++;
    }
    assert(agent1_belief_map_.size() == agent2_belief_map_.size());
}


void BeliefCollisionCheckerBenchmark::runBenchmarks()
{
    /* Create results data structure */
    std::vector<std::pair<double, bool>> chiSquared_results;
    const int num_examples = agent1_belief_map_.size();
    bool result;
    int num_fail = 0;
    double total_time = 0;

    OMPL_INFORM("Testing collision checkers on %d belief pairs.", num_examples);

    // /* Create state validity checker object */
    // auto ChiSquaredPlanValidator = std::make_shared<ChiSquaredBoundaryPVC>(mrmp_pdef_, instance_->getPsafe());
    
    // /* Iterate through all the states, test for collisions, and save results */
    // for (int idx = 0; idx != num_examples; idx++) {
    //     auto start_time = std::chrono::system_clock::now();
    //     result = ChiSquaredPlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
    //     if (!result)
    //         num_fail++;
    //     std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
    //     double execution_time = elapsed_time.count();
    //     total_time += execution_time;
    //     std::pair<double, bool> p(execution_time, result);
    //     chiSquared_results.push_back(p);
    // }

    // double stat = (double)num_fail / num_examples;
    // OMPL_INFORM("%s: rejected %0.1f percent of belief pairs in %0.3f seconds", ChiSquaredPlanValidator->getName().c_str(), (stat * 100.0), total_time);

    // exportResults_("chi_squared_results.csv", chiSquared_results);

    // /* Create results data structure */
    // std::vector<std::pair<double, bool>> sumBlackmore_results;
    // num_fail = 0;
    // total_time = 0;

    // /* Create state validity checker object */
    // auto MinkowskiSumBlackmorePlanValidator = std::make_shared<MinkowskiSumBlackmorePVC>(mrmp_pdef_, instance_->getPsafe());
    
    // /* Iterate through all the states, test for collisions, and save results */
    // for (int idx = 0; idx != num_examples; idx++) {
    //     auto start_time = std::chrono::system_clock::now();
    //     result = MinkowskiSumBlackmorePlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
    //     if (!result)
    //         num_fail++;
    //     std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
    //     double execution_time = elapsed_time.count();
    //     total_time += execution_time;
    //     std::pair<double, bool> p(execution_time, result);
    //     sumBlackmore_results.push_back(p);
    // }

    // stat = (double)num_fail / num_examples;
    // OMPL_INFORM("%s: rejected %0.1f percent of belief pairs in a total of %0.3f seconds", MinkowskiSumBlackmorePlanValidator->getName().c_str(), (stat * 100.0), total_time);

    // exportResults_("minkowski_sum_results.csv", sumBlackmore_results);

    // /* Create results data structure */
    // std::vector<std::pair<double, bool>> Blackmore2_results;
    // num_fail = 0;
    // total_time = 0;

    // /* Create state validity checker object */
    // auto Blackmore2PlanValidator = std::make_shared<Blackmore2PVC>(mrmp_pdef_, instance_->getPsafe());
    
    // /* Iterate through all the states, test for collisions, and save results */
    // for (int idx = 0; idx != num_examples; idx++) {
    //     auto start_time = std::chrono::system_clock::now();
    //     result = Blackmore2PlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
    //     if (!result)
    //         num_fail++;
    //     std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
    //     double execution_time = elapsed_time.count();
    //     total_time += execution_time;
    //     std::pair<double, bool> p(execution_time, result);
    //     Blackmore2_results.push_back(p);
    // }

    // stat = (double)num_fail / num_examples;
    // OMPL_INFORM("%s: rejected %0.1f percent of belief pairs in a total of %0.3f seconds", Blackmore2PlanValidator->getName().c_str(), (stat * 100.0), total_time);

    // exportResults_("blackmore_2_results.csv", Blackmore2_results);

    // /* Create results data structure */
    // std::vector<std::pair<double, bool>> boundingBox_results;
    // num_fail = 0;
    // total_time = 0;

    // /* Create state validity checker object */
    // auto BoundingBoxBlackmorePlanValidator = std::make_shared<BoundingBoxBlackmorePVC>(mrmp_pdef_, instance_->getPsafe());
    
    // /* Iterate through all the states, test for collisions, and save results */
    // for (int idx = 0; idx != num_examples; idx++) {
    //     auto start_time = std::chrono::system_clock::now();
    //     result = BoundingBoxBlackmorePlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
    //     if (!result)
    //         num_fail++;
    //     std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
    //     double execution_time = elapsed_time.count();
    //     total_time += execution_time;
    //     std::pair<double, bool> p(execution_time, result);
    //     boundingBox_results.push_back(p);
    // }

    // stat = (double)num_fail / num_examples;
    // OMPL_INFORM("%s: rejected %0.1f percent of belief pairs in a total of %0.3f seconds", BoundingBoxBlackmorePlanValidator->getName().c_str(), (stat * 100.0), total_time);

    // exportResults_("bounding_box_blackmore_results.csv", boundingBox_results);

    // /* Create results data structure */
    // std::vector<std::pair<double, bool>> adaptiveBlackmore_results;
    // num_fail = 0;
    // total_time = 0;

    // /* Create state validity checker object */
    // auto AdaptiveBlackmorePlanValidator = std::make_shared<AdaptiveRiskBlackmorePVC>(mrmp_pdef_, instance_->getPsafe());
    
    // /* Iterate through all the states, test for collisions, and save results */
    // for (int idx = 0; idx != num_examples; idx++) {
    //     auto start_time = std::chrono::system_clock::now();
    //     result = AdaptiveBlackmorePlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
    //     if (!result)
    //         num_fail++;
    //     std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
    //     double execution_time = elapsed_time.count();
    //     total_time += execution_time;
    //     std::pair<double, bool> p(execution_time, result);
    //     adaptiveBlackmore_results.push_back(p);
    // }

    // stat = (double)num_fail / num_examples;
    // OMPL_INFORM("%s: rejected %0.1f percent of belief pairs in a total of %0.3f seconds", AdaptiveBlackmorePlanValidator->getName().c_str(), (stat * 100.0), total_time);

    // exportResults_("adaptive_blackmore_results.csv", adaptiveBlackmore_results);

    /* Create results data structure */
    std::vector<std::pair<double, bool>> cdfGrid2_results;
    num_fail = 0;
    total_time = 0;

    /* Create state validity checker object */
    auto CDFGrid2PlanValidator = std::make_shared<CDFGridPVC>(mrmp_pdef_, instance_->getPsafe(), 2);

    /* Iterate through all the states, test for collisions, and save results */
    for (int idx = 0; idx != num_examples; idx++) {
        auto start_time = std::chrono::system_clock::now();
        result = CDFGrid2PlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
        if (!result)
            num_fail++;
        std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
        double execution_time = elapsed_time.count();
        total_time += execution_time;
        std::pair<double, bool> p(execution_time, result);
        cdfGrid2_results.push_back(p);
    }

    double stat = (double)num_fail / num_examples;
    OMPL_INFORM("%s: rejected %0.1f percent of belief pairs in a total of %0.3f seconds", CDFGrid2PlanValidator->getName().c_str(), (stat * 100.0), total_time);

    exportResults_("cdfGrid(2)_results.csv", cdfGrid2_results);

    /* Create results data structure */
    std::vector<std::pair<double, bool>> cdfGrid5_results;
    num_fail = 0;
    total_time = 0;

    /* Create state validity checker object */
    auto CDFGrid5PlanValidator = std::make_shared<CDFGridPVC>(mrmp_pdef_, instance_->getPsafe(), 5);

    /* Iterate through all the states, test for collisions, and save results */
    for (int idx = 0; idx != num_examples; idx++) {
        auto start_time = std::chrono::system_clock::now();
        result = CDFGrid5PlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
        if (!result)
            num_fail++;
        std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
        double execution_time = elapsed_time.count();
        total_time += execution_time;
        std::pair<double, bool> p(execution_time, result);
        cdfGrid5_results.push_back(p);
    }

    stat = (double)num_fail / num_examples;
    OMPL_INFORM("%s: rejected %0.1f percent of belief pairs in a total of %0.3f seconds", CDFGrid5PlanValidator->getName().c_str(), (stat * 100.0), total_time);

    exportResults_("cdfGrid(5)_results.csv", cdfGrid5_results);

    /* Create results data structure */
    std::vector<std::pair<double, bool>> cdfGrid20_results;
    num_fail = 0;
    total_time = 0;

    /* Create state validity checker object */
    auto CDFGrid20PlanValidator = std::make_shared<CDFGridPVC>(mrmp_pdef_, instance_->getPsafe(), 20);

    /* Iterate through all the states, test for collisions, and save results */
    for (int idx = 0; idx != num_examples; idx++) {
        auto start_time = std::chrono::system_clock::now();
        result = CDFGrid20PlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
        if (!result)
            num_fail++;
        std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
        double execution_time = elapsed_time.count();
        total_time += execution_time;
        std::pair<double, bool> p(execution_time, result);
        cdfGrid20_results.push_back(p);
        // break;
    }

    stat = (double)num_fail / num_examples;
    OMPL_INFORM("%s: rejected %0.1f percent of belief pairs in a total of %0.3f seconds", CDFGrid20PlanValidator->getName().c_str(), (stat * 100.0), total_time);

    exportResults_("cdfGrid(20)_results.csv", cdfGrid20_results);

    /* Create results data structure */
    std::vector<std::pair<double, bool>> cdfGrid50_results;
    num_fail = 0;
    total_time = 0;

    /* Create state validity checker object */
    auto CDFGrid50PlanValidator = std::make_shared<CDFGridPVC>(mrmp_pdef_, instance_->getPsafe(), 50);

    /* Iterate through all the states, test for collisions, and save results */
    for (int idx = 0; idx != num_examples; idx++) {
        auto start_time = std::chrono::system_clock::now();
        result = CDFGrid50PlanValidator->independentCheck(agent1_belief_map_[idx], agent2_belief_map_[idx]);
        if (!result)
            num_fail++;
        std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
        double execution_time = elapsed_time.count();
        total_time += execution_time;
        std::pair<double, bool> p(execution_time, result);
        cdfGrid50_results.push_back(p);
    }

    stat = (double)num_fail / num_examples;
    OMPL_INFORM("%s: rejected %0.1f percent of belief pairs in a total of %0.3f seconds", CDFGrid50PlanValidator->getName().c_str(), (stat * 100.0), total_time);

    exportResults_("cdfGrid(50)_results.csv", cdfGrid50_results);
}

void BeliefCollisionCheckerBenchmark::exportResults_(std::string filename, std::vector<std::pair<double, bool>> results)
{
    std::ofstream myfile(filename);
    myfile << "Computation Time (s), Success (boolean)\n";
    for (auto itr = results.begin(); itr != results.end(); itr++) {
        myfile << (*itr).first << "," << (*itr).second << std::endl;
    }
    myfile.close();
}






