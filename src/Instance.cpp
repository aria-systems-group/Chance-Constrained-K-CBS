#include "Instance.h"


Instance::Instance(po::variables_map &vm, std::string name): 
    name_(name), 
    mrmp_planner_(vm["solver"].as<std::string>()),
    low_level_planner_(vm["lowlevel"].as<std::string>()),
    x_max_(-1), y_max_(-1), 
    num_agents_(vm["numAgents"].as<int>()),
    map_fpath_(vm["map"].as<std::string>()),
    scen_fpath_(vm["scen"].as<std::string>()),
    p_safe_(vm["p_safe"].as<double>()),
    pvc_(vm["pvc"].as<std::string>()),
    svc_(vm["svc"].as<std::string>())
{
    if (mrmp_planner_ == "K-CBS") {
        // divide p_coll amongst PVC & SVC
        const double p_coll = 1 - p_safe_;
        const double p_coll_obs = p_coll / 2;
        const double p_coll_agnts = p_coll / 2;
        p_safe_agnts_ = 1 - p_coll_agnts;
        p_safe_obs_ = 1 - p_coll_obs;
    }
    bool succ = load_map_();
    if (!succ) {
        OMPL_ERROR("%s: Unable to load map.", name_.c_str());
        exit(-1);
    }
    succ = load_agents_();
    if (!succ) {
        OMPL_ERROR("%s: Unable to load scen file.", name_.c_str());
        exit(-1);
    }
}

Instance::Instance(Instance &other): 
    name_(other.name_), 
    mrmp_planner_(other.mrmp_planner_),
    low_level_planner_(other.low_level_planner_),
    x_max_(other.x_max_), y_max_(other.y_max_), 
    num_agents_(other.num_agents_),
    map_fpath_(other.map_fpath_),
    scen_fpath_(other.scen_fpath_),
    p_safe_(other.p_safe_) 
{
    this->obstacles_ = other.obstacles_;
}

bool Instance::load_map_()
{    
    std::ifstream myfile(map_fpath_);
    if (!myfile.is_open())
        return false;
    std::string line;
    boost::tokenizer< boost::char_separator<char> >::iterator beg;
    getline(myfile, line);
    if (line[0] == 't') // standard benchmark
    {
        boost::char_separator<char> sep(" ");
        getline(myfile, line);
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        beg = tok.begin();
        beg++;
        y_max_ = atoi((*beg).c_str()) - 1; // read number of rows (i.e. y max value)
        getline(myfile, line);
        boost::tokenizer< boost::char_separator<char> > tok2(line, sep);
        beg = tok2.begin();
        beg++;
        x_max_ = atoi((*beg).c_str()) - 1; // read number of cols (i.e. x max value)
        getline(myfile, line); // skip "map"
    }
    assert((x_max_ > 0 && y_max_ > 0));
    // getline(myfile, line);
    int c = 0;
    while (getline(myfile, line)) {
        for (int r = 0; r < x_max_; r++) {
            if (line[r] != '.') {
                obstacles_.emplace_back(new RectangularObstacle(r, c, 1, 1));
            }
        }
        c++; // pun intended...
    }
    myfile.close();
    return true;
}

bool Instance::load_agents_()
{
    std::ifstream myfile(scen_fpath_);
    if (!myfile.is_open())
        return false;
    std::string line;
    getline(myfile, line);
    if (num_agents_ == 0)
    {
        OMPL_ERROR("%s: The number of agents should be larger than 0", name_.c_str());
        exit(-1);
    }
    boost::char_separator<char> sep("\t");
    for (int i = 0; i < num_agents_; i++)
    {
        // extract the start Location and goal Location for robot i
        getline(myfile, line);
        if (line.empty())
        {
            OMPL_ERROR("%s: The instance has only %d robots.", (name_.c_str(), i));
            exit(-1);
        }
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
        beg++; // skip the first number
        beg++; // skip the map name
        beg++; // skip the columns
        beg++; // skip the rows
        int row = atoi((*beg).c_str());
        beg++;
        int col = atoi((*beg).c_str());
        Location start(row, col);
        beg++;
        row = atoi((*beg).c_str());
        beg++;
        col = atoi((*beg).c_str());
        Location goal(row, col);
        beg++;
        std::string shape = std::string(*beg);
        beg++;
        std::string dyn_model = std::string(*beg);

        std::string name = "Robot " + std::to_string(i);

        // add robot to list
        if (shape == "Point")
            robots_.emplace_back(new PointRobot(name, dyn_model, start, goal));
        else if (shape == "Rectangle")
            robots_.emplace_back(new RectangularRobot(name, dyn_model, start, goal, 0.25, 0.25)); // manual size To-Do!
        else
            OMPL_WARN("%s: %sRobot class not yet implemented!", name_.c_str());        
    }
    myfile.close();
    return true;
}

// printing methods for usability
void Instance::printObstacles()
{
    OMPL_INFORM("%d Obstacle(s) [Center, Size]: ", obstacles_.size());
    for (auto o_itr = obstacles_.begin(); o_itr < obstacles_.end(); o_itr++)
    {
        OMPL_INFORM("- Obstacle: ");
        (*o_itr)->printPoints();
    }
};

void Instance::printRobots()
{
    OMPL_INFORM("%d Robot: ", robots_.size());
    for (auto itr = robots_.begin(); itr < robots_.end(); itr++)
    {
        OMPL_INFORM("   - Name: %s", (*itr)->getName().c_str());
        OMPL_INFORM("   - Dynamics: %s", (*itr)->getDynamicsModel().c_str());
        (*itr)->printShape();
        OMPL_INFORM("   - Goal: (%0.1f,%0.1f)", (*itr)->getGoalLocation().x_, (*itr)->getGoalLocation().y_);
    }
};
void Instance::print()
{
    printDimensions();
    printObstacles();
    printRobots();
};
