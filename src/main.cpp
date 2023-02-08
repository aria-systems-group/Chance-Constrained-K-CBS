#include "OmplSetUp.h"
#include "Mergers/DeterministicMerger.h"
#include "Mergers/BeliefMerger.h"
#include "PlanValidityCheckers/DeterministicPlanValidityChecker.h"
#include "PlanValidityCheckers/BeliefPlanValidityChecker.h"
#include "Planners/KCBS.h"
#include "postProcess.h"

// OMPL_INFORM("OMPL version: %s", OMPL_VERSION);  // blue font
// OMPL_WARN("OMPL version: %s", OMPL_VERSION);  // yellow font
// OMPL_ERROR("OMPL version: %s", OMPL_VERSION); // red font
// these follow syntax of printf see (https://www.cplusplus.com/reference/cstdio/printf/)

void parse_cmd_line(int &argc, char ** &argv, po::variables_map &vm, po::options_description &desc)
{
    desc.add_options()
        ("help,h", "produce help message")

        // params for the input instance and experiment settings
        ("map,m", po::value<std::string>()->required(), "the *.map file")
        ("scen,s", po::value<std::string>()->required(), "the *.scen file")
        ("numAgents,k", po::value<int>()->required(), "number of agents inside instance")
        ("solver", po::value<std::string>()->default_value("K-CBS"), "the high-level MRMP solver (K-CBS, PBS, MR-RRT)")
        ("lowlevel,l", po::value<std::string>()->default_value("RRT"), "The low-level motion planner for K-CBS (RRT, BSST)")
        ("bound,b", po::value<int>()->default_value(std::numeric_limits<int>::max()), "The merge bound of K-CBS.")
        ("time,t", po::value<double>()->default_value(600), "cutoff time (seconds)")
        ("output,o", po::value<std::string>()->default_value("results"), "output file name (no extension)")
        ("screen", po::value<int>()->default_value(0),
                "screen option \n0 := none \n1 := K-CBS updates \n2 := Low-Level Planner updates \n3 := MRMP detailed updates")
        ;
    po::store(po::parse_command_line(argc, argv, desc), vm);
}

int main(int argc, char ** argv)
{
    po::variables_map vm;
    po::options_description desc("Allowed options");
    parse_cmd_line(argc, argv, vm, desc);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    // set-up planning instance
    InstancePtr instance = std::make_shared<Instance>(vm);
    // instance->print(); // print the map (for debugging etc.)

    // set-up low-level planners
    std::vector<MotionPlanningProblemPtr> mp_problems = set_up_all_MP_Problems(instance);

    // set-up MRMP Problem Definition
    MultiRobotProblemDefinitionPtr mrmp_pdef = std::make_shared<MultiRobotProblemDefinition>(mp_problems);
    mrmp_pdef->setMultiRobotInstance(instance);

    // figure out which type of experiment to run, and run it
    std::string high_level_planner = instance->getPlannerName();
    std::string low_level_planner = instance->getLowLevelPlannerName();
    
    if (high_level_planner == "K-CBS") {
        if (low_level_planner == "RRT") {
            // set-up (and include) a Merger in case merge bound is hit
            MergerPtr merger = std::make_shared<DeterministicMerger>(mrmp_pdef);
            mrmp_pdef->setMerger(merger);
            // set-up (and include) a PlanValidityChecker for agent-to-agent collision checking
            PlanValidityCheckerPtr planValidator = std::make_shared<DeterministicPlanValidityChecker>(mrmp_pdef);
            mrmp_pdef->setPlanValidator(planValidator);
            // create instance of K-CBS, set-up, and solve
            ob::PlannerPtr p(std::make_shared<oc::KCBS>(mrmp_pdef));
            p->as<oc::KCBS>()->setMergeBound(vm["bound"].as<int>());
            bool solved = p->solve(vm["time"].as<double>());
        }
        else if (low_level_planner == "BSST") {
            // set-up (and include) a Merger in case merge bound is hit
            MergerPtr merger = std::make_shared<BeliefMerger>(mrmp_pdef);
            mrmp_pdef->setMerger(merger);
            // set-up (and include) a PlanValidityChecker for agent-to-agent collision checking
            PlanValidityCheckerPtr planValidator = std::make_shared<BeliefPlanValidityChecker>(mrmp_pdef);
            mrmp_pdef->setPlanValidator(planValidator);
            // create instance of K-CBS, set-up, and solve
            ob::PlannerPtr p(std::make_shared<oc::KCBS>(mrmp_pdef));
            p->as<oc::KCBS>()->setMergeBound(vm["bound"].as<int>());
            bool solved = p->solve(vm["time"].as<double>());
            if (solved) {
                // extract and write results to file
                std::vector<oc::PathControl*> plan;
                for (int i=0; i < vm["numAgents"].as<int>(); i++) {
                    oc::PathControl* path = mrmp_pdef->getRobotProblemDefinitionPtr(i)->getSolutionPath()->as<oc::PathControl>();
                    std::cout << path << std::endl;
                    plan.push_back(path);
                }
                exportBeliefPlan(plan, vm["output"].as<std::string>());
            }
        } 
        else {
            OMPL_ERROR("%s: Implementation of K-CBS w/ %s is unavailable.", "main", low_level_planner.c_str());
        }
    }
    else if (high_level_planner == "PBS") {
        if (low_level_planner == "RRT") {
            // solve standard MRMP w. PBS
            OMPL_WARN("%s: Planning with PBS not implemented after refactor.", "main");
        }
        else {
            OMPL_ERROR("%s: Implementation of PBS w/ %s is unavailable.", "main", low_level_planner.c_str());
        }
    }
    else if (high_level_planner == "MR-RRT") {
        // solve standard MRMP w. MR-RRT
        OMPL_WARN("%s: Planning with MR-RRT not implemented after refactor.", "main");
    }
    else {
        OMPL_ERROR("%s: Implementation of %s w/ %s is unavailable.", "main", high_level_planner.c_str());
    }
    return 0;
}
