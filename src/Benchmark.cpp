#include "includes/Benchmark.h"

dataStruct benchmark(const std::string problem, const double planningTime, 
    const int benchType, const int mergParam, const std::string bypass)
{
    /* from problem file, create worlds and ompl problem instances */
    World *decentralized_w = yaml2world(problem);
    std::string centralized_problem = problem;
    centralized_problem.resize(centralized_problem.size() - 4);
    centralized_problem = centralized_problem + "_centralized.yml";
    if (benchType == 3)
    {
        if (std::filesystem::exists(centralized_problem))
        {
            World *centralized_w = yaml2world(centralized_problem);

            /* initialize problems */
            const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
                std::shared_ptr<ob::ProblemDefinition>>> decentralized_ompl_problem = multiAgentSetUp(decentralized_w);

            const std::pair<std::shared_ptr<oc::SpaceInformation>, 
                std::shared_ptr<ob::ProblemDefinition>> centralized_ompl_problem = multiAgentSetUp(centralized_w)[0];

            /* initialize planners */
            // MA-RRT
            oc::MA_RRT *ma_rrt = new oc::MA_RRT(centralized_ompl_problem.first);
            ma_rrt->setProblemDefinition(centralized_ompl_problem.second);
            ma_rrt->setup();
            ob::PlannerPtr ma_rrt_planner(ma_rrt);

            // PBS
            oc::PBS *pbs = new oc::PBS(decentralized_ompl_problem); // PBS
            pbs->setWorld(decentralized_w);
            ob::PlannerPtr pbs_planner(pbs);

            // K-CBS
            oc::KD_CBS *k_cbs = new oc::KD_CBS(decentralized_ompl_problem); // K_CBS
            k_cbs->setWorld(decentralized_w);
            k_cbs->setMergeBound(mergParam);
            if (bypass == "True")
                k_cbs->performBypassing();
            ob::PlannerPtr k_cbs_planner(k_cbs);

            /* initialize data columns */
            std::vector<std::string> planner_col{ma_rrt_planner->getName(), pbs_planner->getName(), k_cbs_planner->getName()};
            std::vector<std::string> isSolved_col{};
            std::vector<std::string> compTime_col{};
            std::vector<std::string> mergers{};


            OMPL_INFORM("******** Beginning Benchmark ******** \n");
            /* test MA_RRT */
            // plan
            bool ma_rrt_solved = ma_rrt_planner->solve(planningTime);
            // record success
            if (ma_rrt_solved)
                isSolved_col.push_back("1");
            else
                isSolved_col.push_back("0");
            // record computation time
            compTime_col.push_back(std::to_string(ma_rrt_planner->as<oc::MA_RRT>()->getSolveTime()));

            // record merger
            mergers.push_back("");

            /* test PBS */
            // plan
            bool pbs_solved = pbs_planner->solve(planningTime);
            // record success
            if (pbs_solved)
                isSolved_col.push_back("1");
            else
                isSolved_col.push_back("0");
            // record computation time
            compTime_col.push_back(std::to_string(pbs_planner->as<oc::PBS>()->getSolveTime()));

            // record merger
            mergers.push_back("");

            /* test K-CBS */
            // plan
            bool k_cbs_solved = k_cbs_planner->solve(planningTime);
            // record success
            if (k_cbs_solved)
                isSolved_col.push_back("1");
            else
                isSolved_col.push_back("0");
            
            // record computation time
            compTime_col.push_back(std::to_string(k_cbs_planner->as<oc::KD_CBS>()->getSolveTime()));

            // record merger list
            std::vector<std::pair<int, int>> merger_list = k_cbs_planner->as<oc::KD_CBS>()->getMergers();
            std::string list = "";
            for (auto merge: merger_list)
            {
                list = list + "(" + std::to_string(merge.first) + " " + std::to_string(merge.second) + ") ";
            }
            mergers.push_back(list);

            // col 1: planner names
            std::pair< std::string, std::vector<std::string> > planners{"Algorithm", planner_col};
            // col 2: success
            std::pair< std::string, std::vector<std::string> > solved{"Success (Boolean)", isSolved_col};
            // col 3: computation time
            std::pair< std::string, std::vector<std::string> > times{"Computation Time (s)", compTime_col};
            // col 4: mergers
            std::pair< std::string, std::vector<std::string> > merging{"Mergers", mergers};

            // put it all together
            dataStruct data{planners, solved, times, merging};
            OMPL_INFORM("******** Benchmark Complete ******** \n");
            return data;
        }
        else
        {
            OMPL_WARN("No Centrlalized Version of Problem Exists. Ignoring MA-RRT Benchmark.");

            /* initialize problems */
            const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
                std::shared_ptr<ob::ProblemDefinition>>> decentralized_ompl_problem = multiAgentSetUp(decentralized_w);

            /* initialize planners */
            // PBS
            oc::PBS *pbs = new oc::PBS(decentralized_ompl_problem); // PBS
            pbs->setWorld(decentralized_w);
            ob::PlannerPtr pbs_planner(pbs);

            // K-CBS
            oc::KD_CBS *k_cbs = new oc::KD_CBS(decentralized_ompl_problem); // K_CBS
            k_cbs->setWorld(decentralized_w);
            k_cbs->setMergeBound(mergParam);
            if (bypass == "True")
                k_cbs->performBypassing();
            ob::PlannerPtr k_cbs_planner(k_cbs);

            /* initialize data columns */
            std::vector<std::string> planner_col{pbs_planner->getName(), k_cbs_planner->getName()};
            std::vector<std::string> isSolved_col{};
            std::vector<std::string> compTime_col{};
            std::vector<std::string> mergers{};

            OMPL_INFORM("******** Beginning Benchmark ******** \n");
            /* test PBS */
            // plan
            bool pbs_solved = pbs_planner->solve(planningTime);
            // record success
            if (pbs_solved)
                isSolved_col.push_back("1");
            else
                isSolved_col.push_back("0");
            // record computation time
            compTime_col.push_back(std::to_string(pbs_planner->as<oc::PBS>()->getSolveTime()));
            // record merger
            mergers.push_back("");

            /* test K-CBS */
            // plan
            bool k_cbs_solved = k_cbs_planner->solve(planningTime);
            // record success
            if (k_cbs_solved)
                isSolved_col.push_back("1");
            else
                isSolved_col.push_back("0");
            // record computation time
            compTime_col.push_back(std::to_string(k_cbs_planner->as<oc::KD_CBS>()->getSolveTime()));


            // record merger list
            std::vector<std::pair<int, int>> merger_list = k_cbs_planner->as<oc::KD_CBS>()->getMergers();
            std::string list = "";
            for (auto merge: merger_list)
            {
                list = list + "(" + std::to_string(merge.first) + " " + std::to_string(merge.second) + ") ";
            }
            mergers.push_back(list);

            // col 1: planner names
            std::pair< std::string, std::vector<std::string> > planners{"Algorithm", planner_col};
            // col 2: success
            std::pair< std::string, std::vector<std::string> > solved{"Success (Boolean)", isSolved_col};
            // col 3: computation time
            std::pair< std::string, std::vector<std::string> > times{"Computation Time (s)", compTime_col};
            // col 4: mergers
            std::pair< std::string, std::vector<std::string> > merging{"Mergers", mergers};

            // put it all together
            dataStruct data{planners, solved, times, merging};
            OMPL_INFORM("******** Benchmark Complete ******** \n");
            return data;
        }
    }
    else if (benchType == 2)
    {
        /* initialize problems */
        const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
            std::shared_ptr<ob::ProblemDefinition>>> decentralized_ompl_problem = multiAgentSetUp(decentralized_w);

        /* initialize planners */
        // PBS
        oc::PBS *pbs = new oc::PBS(decentralized_ompl_problem); // PBS
        pbs->setWorld(decentralized_w);
        ob::PlannerPtr pbs_planner(pbs);

        // K-CBS
        oc::KD_CBS *k_cbs = new oc::KD_CBS(decentralized_ompl_problem); // K_CBS
        k_cbs->setWorld(decentralized_w);
        k_cbs->setMergeBound(mergParam);
        if (bypass == "True")
            k_cbs->performBypassing();
        ob::PlannerPtr k_cbs_planner(k_cbs);

        /* initialize data columns */
        std::vector<std::string> planner_col{pbs_planner->getName(), k_cbs_planner->getName()};
        std::vector<std::string> isSolved_col{};
        std::vector<std::string> compTime_col{};
        std::vector<std::string> mergers{};

        OMPL_INFORM("******** Beginning Benchmark ******** \n");
        /* test PBS */
        // plan
        bool pbs_solved = pbs_planner->solve(planningTime);
        // record success
        if (pbs_solved)
            isSolved_col.push_back("1");
        else
            isSolved_col.push_back("0");
        // record computation time
        compTime_col.push_back(std::to_string(pbs_planner->as<oc::PBS>()->getSolveTime()));
        // record merger
        mergers.push_back("");

        /* test K-CBS */
        // plan
        bool k_cbs_solved = k_cbs_planner->solve(planningTime);
        // record success
        if (k_cbs_solved)
            isSolved_col.push_back("1");
        else
            isSolved_col.push_back("0");
        // record computation time
        compTime_col.push_back(std::to_string(k_cbs_planner->as<oc::KD_CBS>()->getSolveTime()));

        // record merger list
        std::vector<std::pair<int, int>> merger_list = k_cbs_planner->as<oc::KD_CBS>()->getMergers();
        std::string list = "";
        for (auto merge: merger_list)
        {
            list = list + "(" + std::to_string(merge.first) + " " + std::to_string(merge.second) + ") ";
        }
        mergers.push_back(list);

        // col 1: planner names
        std::pair< std::string, std::vector<std::string> > planners{"Algorithm", planner_col};
        // col 2: success
        std::pair< std::string, std::vector<std::string> > solved{"Success (Boolean)", isSolved_col};
        // col 3: computation time
        std::pair< std::string, std::vector<std::string> > times{"Computation Time (s)", compTime_col};
        // col 4: mergers
        std::pair< std::string, std::vector<std::string> > merging{"Mergers", mergers};

        // put it all together
        dataStruct data{planners, solved, times, merging};
        OMPL_INFORM("******** Benchmark Complete ******** \n");
        return data;
    }
    else if (benchType == 1)
    {
        /* initialize problems */
        const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
            std::shared_ptr<ob::ProblemDefinition>>> decentralized_ompl_problem = multiAgentSetUp(decentralized_w);

        /* initialize planners */

        // K-CBS
        oc::KD_CBS *k_cbs = new oc::KD_CBS(decentralized_ompl_problem); // K_CBS
        k_cbs->setWorld(decentralized_w);
        k_cbs->setMergeBound(mergParam);
        if (bypass == "True")
            k_cbs->performBypassing();
        ob::PlannerPtr k_cbs_planner(k_cbs);

        /* initialize data columns */
        std::vector<std::string> planner_col{k_cbs_planner->getName()};
        std::vector<std::string> isSolved_col{};
        std::vector<std::string> compTime_col{};
        std::vector<std::string> mergers{};

        OMPL_INFORM("******** Beginning Benchmark ******** \n");
        /* test K-CBS */
        // plan
        bool k_cbs_solved = k_cbs_planner->solve(planningTime);
        // record success
        if (k_cbs_solved)
            isSolved_col.push_back("1");
        else
            isSolved_col.push_back("0");
        // record computation time
        compTime_col.push_back(std::to_string(k_cbs_planner->as<oc::KD_CBS>()->getSolveTime()));

        // record merger list
        std::vector<std::pair<int, int>> merger_list = k_cbs_planner->as<oc::KD_CBS>()->getMergers();
        std::string list = "";
        for (auto merge: merger_list)
        {
            list = list + "(" + std::to_string(merge.first) + " " + std::to_string(merge.second) + ") ";
        }
        mergers.push_back(list);

        // col 1: planner names
        std::pair< std::string, std::vector<std::string> > planners{"Algorithm", planner_col};
        // col 2: success
        std::pair< std::string, std::vector<std::string> > solved{"Success (Boolean)", isSolved_col};
        // col 3: computation time
        std::pair< std::string, std::vector<std::string> > times{"Computation Time (s)", compTime_col};
        // col 4: mergers
        std::pair< std::string, std::vector<std::string> > merging{"Mergers", mergers};

        // put it all together
        dataStruct data{planners, solved, times, merging};
        OMPL_INFORM("******** Benchmark Complete ******** \n");
        return data;
    }
    else
    {
        OMPL_ERROR("Invalid benchmark type provided!");
        exit(1);
    }
}


// https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
void write_csv(const char* logName, dataStruct dataset)
{
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size

    OMPL_INFORM("Writing Results to %s \n", logName);

    // Create an output filestream object
    std::ofstream myFile(logName);
    
    // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";

	if (!dataset.empty())
	{ 
    	// Send data to the stream
    	for(int i = 0; i < dataset.at(0).second.size(); ++i)
    	{
    	    for(int j = 0; j < dataset.size(); ++j)
    	    {
    	        myFile << dataset.at(j).second.at(i);
    	        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    	    }
    	    myFile << "\n";
    	}
	}
    // Close the file
    myFile.close();
}




