#include "includes/postProcess.h"

void checkDisjoint(const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition>>> problem,
        std::vector<oc::PathControl> plan, const int begin, int depth, 
    bool &intersection, bool &done)
{
	int numDone = 0;
    // get the states from plan at specified depth
    std::vector<std::vector<const ob::State*>> sysStates(plan.size());
    for (int a = 0; a < plan.size(); a++)
    {
        for (int d = begin; d <= begin+depth; d++)
        {
            // add states to vector (if possible)
            if (d < plan[a].getStateCount())
            {
                sysStates[a].push_back(plan[a].getState(d));
            }
            else
            {
            	numDone++;
            	break;
            }
        }
    }
    if (numDone == sysStates.size())
    	done = true;

    // not done, need to check disjoint
    // printf("Interpolating trajectories...\n");
    // connect states to a 2D line
    std::vector<std::vector<Segment>> sysLines(plan.size());
    for (int a = 0; a < sysStates.size(); a++)
    {
        const ob::StateSpace *space(problem[a].first->getStateSpace().get());
        // std::cout << sysStates[a].size() << std::endl;
        // std::cout << depth << std::endl;
        for (int i = 1; i < sysStates[a].size(); i++)
        {
        	// problem[a].first->printState(sysStates[a][i]);
            // create line from (i-1)-th to i-th index
            const ob::State *st_i_m_1 = sysStates[a][i-1];
            std::vector<double> reals_i_m_1;
            space->copyToReals(reals_i_m_1, st_i_m_1);
            Point pt_i_m_1(reals_i_m_1[0], reals_i_m_1[1]);

            const ob::State *st_i = sysStates[a][i];
            std::vector<double> reals_i;
            space->copyToReals(reals_i, st_i);
            Point pt_i(reals_i[0], reals_i[1]);

            Segment line(pt_i_m_1, pt_i);

            sysLines[a].push_back(line);
        }
    }
    // printf("Checking lines are disjoint... \n");
    for (int a1 = 0; a1 < sysLines.size(); a1++)
    {
        for (int a2 = 0; a2 < sysLines.size(); a2++)
        {
            if (a1 != a2)
            {
                // check sysLines[a1] vs sysLines[a2]
                for (const Segment s1: sysLines[a1])
                {
                    for (const Segment s2: sysLines[a2])
                    {
                        // printf("Agent: %i Line: (%.1f, %.1f) to (%.1f, %.1f) \n", 
                        // 	0, bg::get<0, 0>(s1), bg::get<0, 1>(s1),
                        // 	bg::get<1, 0>(s1), bg::get<1, 1>(s1));
                        // printf("Agent: %i Line: (%.1f, %.1f) to (%.1f, %.1f) \n", 
                        // 	1, bg::get<0, 0>(s2), bg::get<0, 1>(s2),
                        // 	bg::get<1, 0>(s2), bg::get<1, 1>(s2));
                        intersection = boost::geometry::intersects(s1, s2);
                        if (intersection == true)
                        {
                            // printf("intersection occurs \n");
                            return;
                        }
                    }
                }
            }
        }
    }
}

std::vector<int> generateExplanation(const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition>>> problem)
{
    printf("Generating Explanation...\n");
    std::vector<oc::PathControl> plan;
    int maxLength = 0;
    for (int i = 0; i < problem.size(); i++)
    {
        oc::PathControl traj = static_cast<oc::PathControl &>
            (*problem[i].second->getSolutionPath());
        traj.interpolate();
        // traj.printAsMatrix(std::cout);
        if (traj.getStateCount() > maxLength)
        	maxLength = traj.getStateCount();
        plan.push_back(traj);
    }
    std::vector<int> costVec;
    int cost = 1;
    int beginIdx = 0;
    int depth = 1;
    bool done = false;

    while(!done)
    {
        if (depth % 100 == 0)
            printf("Depth: %i \n", depth);
        bool intersection = false;
        checkDisjoint(problem, plan, beginIdx, depth, intersection, done);
        if (intersection == true && !done)
        {
            for (int d = 1; d < depth; d++)
            	costVec.push_back(cost);
            cost++;
            beginIdx = beginIdx + depth - 1;
            depth = 1;
        }
        else if (intersection == false && !done)
        {
            depth ++;
        }
        else if (done)
        {
            while (costVec.size() < maxLength)
            	costVec.push_back(cost);
            done = true;
        }
    }
    printf("Done. Explanation contains %i images.\n", costVec.back());
    return costVec;
}



// generate date/time information to solutions or solution directories
std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y_%m_%d_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '_');
    return s;
}

// parent function for including date/time information to files
fs::path appendTimeToFileName(const fs::path& fileName)
{
    return fileName.stem().string() + "_" + GetCurrentTimeForFileName() + fileName.extension().string();
}

// write solultion to the system
void write2sys(const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition>>> problem, 
        const std::vector<Agent*> agents, const std::string problem_name,
        std::vector<int> costs)
{
    fs::path sol_dir = "solutions/" + problem_name;
    fs::create_directories(sol_dir);
    for (int i = 0; i < problem.size(); i++)
    {
        std::string fileName = agents[i]->getName() + ".txt";
        auto filePath = fs::current_path() / sol_dir / fs::path(fileName); // appendTimeToFileName(fileName); // e.g. MyPrettyFile_2018-06-09_01-42-00.txt
        std::ofstream file(filePath);
        oc::PathControl traj = static_cast<oc::PathControl &>
            (*problem[i].second->getSolutionPath());
        traj.interpolate();
        traj.printAsMatrix(file);
        file.close();
        // go back to file and add cost to end of each line
        std::ifstream in(filePath);
        auto filePath2 = fs::current_path() / sol_dir / 
            fs::path(agents[i]->getName() + "_explanation.txt"); // appendTimeToFileName(fileName); // e.g. MyPrettyFile_2018-06-09_01-42-00.txt
		std::ofstream out(filePath2);
		std::string line;

		int idx = 0;
		while (getline(in, line))
		{
		    if (!line.empty())
		    {
		        out << line;
		
		        // process line as needed...
		
		        out << " " << costs[idx];
		    }
		    out << "\n";
		    idx++;
		}

in.close();
out.close();
    }
}