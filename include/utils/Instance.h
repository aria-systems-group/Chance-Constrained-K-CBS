#pragma once
#include "utils/common.h"
#include <filesystem>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/program_options.hpp>

namespace fs = std::filesystem;
namespace po = boost::program_options;


// world class holds all relevent data in the world that is used by OMPL
OMPL_CLASS_FORWARD(Instance);
class Instance
{
public:
    Instance(Instance &other);
    Instance(po::variables_map &vm, std::string name = "Instance");
    // methods for dimensions
    const double getPsafe() const {return p_safe_;};
    const double getPsafeObs() const {return p_safe_obs_;};
    const double getPsafeAgents() const {return p_safe_agnts_;};
    const std::string getPVC() const {return pvc_;};
    const std::string getSVC() const {return svc_;};
    std::vector<double> getDimensions() const {return {x_max_, y_max_};};
    void printDimensions(){OMPL_INFORM("Space Dimensions: \n"
        "x: [0, %0.2f] \n"
        "y: [0, %0.2f]", x_max_, y_max_);};
    std::vector<Obstacle*> getObstacles() const {return obstacles_;};
    std::vector<Robot*> getRobots() const {return robots_;};
    const std::string getPlannerName() const {return mrmp_planner_;};
    const std::string getLowLevelPlannerName() const {return low_level_planner_;};
    // printing methods for usability
    void printObstacles();
    void printRobots();
    void print();
private:
    bool load_map_();
    bool load_agents_();
    double x_max_;
    double y_max_;
    double p_safe_agnts_ = -1;
    double p_safe_obs_ = -1;
    const int num_agents_;
    const double p_safe_;
    const fs::path map_fpath_;
    const fs::path scen_fpath_;
    const std::string name_;
    const std::string pvc_;
    const std::string svc_;
    const std::string mrmp_planner_;
    const std::string low_level_planner_;
    std::vector<Obstacle*> obstacles_;
    std::vector<Robot*> robots_;
};
