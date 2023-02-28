#include "utils/Benchmark.h"


void run_kcbs_benchmark(InstancePtr mrmp_instance, const double merge_bound, const double comp_time, std::string filename)
{
    // set-up low-level planners
    std::vector<MotionPlanningProblemPtr> mp_problems = set_up_all_MP_Problems(mrmp_instance);
    // set-up MRMP Problem Definition
    MultiRobotProblemDefinitionPtr mrmp_pdef = std::make_shared<MultiRobotProblemDefinition>(mp_problems);
    mrmp_pdef->setMultiRobotInstance(mrmp_instance);
    const std::string low_level_planner = mrmp_instance->getLowLevelPlannerName();

    // set-up K-CBS based on planning type and current settings
    if (low_level_planner == "RRT") {
        // set-up (and include) a Merger in case merge bound is hit
        MergerPtr merger = std::make_shared<DeterministicMerger>(mrmp_pdef);
        mrmp_pdef->setMerger(merger);
        // set-up (and include) a PlanValidityChecker for agent-to-agent collision checking
        PlanValidityCheckerPtr planValidator = std::make_shared<DeterministicPlanValidityChecker>(mrmp_pdef);
        mrmp_pdef->setPlanValidator(planValidator);
    }
    else if (low_level_planner == "BSST") {
        // set-up (and include) a Merger in case merge bound is hit
        MergerPtr merger = std::make_shared<BeliefMerger>(mrmp_pdef);
        mrmp_pdef->setMerger(merger);
        // set-up (and include) a PlanValidityChecker for agent-to-agent collision checking
        PlanValidityCheckerPtr planValidator = nullptr;
        if (mrmp_instance->getPVC() == "ChiSquared") {
            planValidator = std::make_shared<ChiSquaredBoundaryPVC>(mrmp_pdef, mrmp_instance->getPsafeAgents());
        }
        else if (mrmp_instance->getPVC() == "Blackmore") {
            planValidator = std::make_shared<MinkowskiSumBlackmorePVC>(mrmp_pdef, mrmp_instance->getPsafeAgents());
        }
        else if (mrmp_instance->getPVC() == "AdaptiveBlackmore") {
            planValidator = std::make_shared<AdaptiveRiskBlackmorePVC>(mrmp_pdef, mrmp_instance->getPsafeAgents());
        }
        else if (mrmp_instance->getPVC().find("CDFGrid") != std::string::npos) {
            boost::char_separator<char> sep("-");
            boost::tokenizer< boost::char_separator<char> > tok(mrmp_instance->getPVC(), sep);
            boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
            beg++;
            const int disks = atoi((*beg).c_str());
            OMPL_INFORM("The plan validity checker is CDFGrid with discretization of %d", disks);
            planValidator = std::make_shared<CDFGridPVC>(mrmp_pdef, mrmp_instance->getPsafeAgents(), disks);
        }
        else {
            OMPL_ERROR("Plan Validity Checker ``%s`` is not available.", mrmp_instance->getPVC().c_str());
        }
        mrmp_pdef->setPlanValidator(planValidator);
    }

    std::vector<std::tuple<bool, double, double>> results; // solved, computation time, path length

    for (int i = 0; i < 50; i++) {
        // create K-CBS instance
        ob::PlannerPtr p(std::make_shared<oc::KCBS>(mrmp_pdef));
        p->as<oc::KCBS>()->setMergeBound(merge_bound);
        // plan with K-CBS
        bool solved = p->solve(comp_time);
        // fill results
        std::tuple<bool, double, double> r{solved, p->as<oc::KCBS>()->getComputationTime(), p->as<oc::KCBS>()->getSolutionSOC()};
        results.push_back(r);
        // update results file with n
        write_csv(filename, r);
        // clear memory
        p.reset();
        auto all_pdefs = mrmp_pdef->getAllProblemInformation();
        for (auto pdef_itr = all_pdefs.begin(); pdef_itr != all_pdefs.end(); pdef_itr++) {
            (*pdef_itr)->getProblemDefinition()->clearSolutionPaths();// << std::endl; //->clearSolutionPaths()
        }
    }
}

void run_centralized_bsst_benchmark(InstancePtr mrmp_instance, const double comp_time, std::string filename)
{
    // set-up low-level planners
    std::vector<MotionPlanningProblemPtr> mp_problems = set_up_all_MP_Problems(mrmp_instance);
    // set-up MRMP Problem Definition
    MultiRobotProblemDefinitionPtr mrmp_pdef = std::make_shared<MultiRobotProblemDefinition>(mp_problems);
    mrmp_pdef->setMultiRobotInstance(mrmp_instance);
    const std::string low_level_planner = mrmp_instance->getLowLevelPlannerName();

    std::vector<std::tuple<bool, double, double>> results; // solved, computation time, path length

    for (int i = 0; i < 50; i++) {
        // solve with CentralizedBSST instance
        PlannerPtr p = mrmp_pdef->getRobotMotionPlanningProblemPtr(0)->getPlanner();
        bool solved = p->solve(comp_time);
        // fill results
        std::tuple<bool, double, double> r{solved, p->as<oc::CentralizedBSST>()->getComputationTime(), p->as<oc::CentralizedBSST>()->getSolutionSOC()};
        results.push_back(r);
        // update results file with n
        write_csv(filename, r);
        // clear memory
        mrmp_pdef->getRobotMotionPlanningProblemPtr(0)->getPlanner()->clear();
        auto all_pdefs = mrmp_pdef->getAllProblemInformation();
        for (auto pdef_itr = all_pdefs.begin(); pdef_itr != all_pdefs.end(); pdef_itr++) {
            (*pdef_itr)->getProblemDefinition()->clearSolutionPaths();// << std::endl; //->clearSolutionPaths()
        }
    }
}

void write_csv(std::string filename, std::tuple<bool, double, double> results)
{
    // Make a CSV file with one or more columns of integer values
    std::ifstream infile(filename);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        std::ofstream addHeads(filename);
        addHeads << "Success (Boolean),Computation Time (s), Sum of Controls (double)" << std::endl;
        addHeads.close();
    }
    std::ofstream stats(filename, std::ios::app);
    stats << std::get<0>(results) << "," << std::get<1>(results) << "," << std::get<2>(results);
    stats << std::endl;
}
