/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all relevent code for converting the problem 
* file (*.yml) to a World Object 
*********************************************************************/
 
/* Author: Justin Kottinger */

#pragma once
#include <yaml.h>


// rectangular Obstacle defined by two points (xMin, yMin) and (xMax, yMax)
struct Obstacle {
    double xMin_;
    double yMin_;
    double xMax_;
    double yMax_;
};

// An agent has a name, shape, dynamics, start and goal regions
// Created as class to keep important variables safe
class Agent
{
public:
    Agent(std::string name, std::string dyn, std::vector<double> shape, std::vector<double> s, std::vector<double> g) {
        name_ = name;
        dynamics_ = dyn;
        shape_ = shape;
        start_ = s;
        goal_ = g;
    }
    std::string getName() const {return name_;};
    std::string getDynamics() const {return dynamics_;};
    std::vector<double> getShape() const {return shape_;};
    std::vector<double> getStartLocation() const {return start_;};
    std::vector<double> getGoalLocation() const {return goal_;};
private:
    std::string name_;
    std::string dynamics_;
    std::vector<double> shape_;
    std::vector<double> start_;
    std::vector<double> goal_;
};



// world class holds all relevent data in the world that is used by OMPL
class World
{
public:
    World(){}
    // methods for dimensions
    void setWorldDimensions(int x, int y){xDim_ = x; yDim_ = y;};
    std::vector<double> getWorldDimensions() const {return {xDim_, yDim_};};
    void printWorldDimensions(){OMPL_INFORM("Space Dimensions: [%0.2f, %0.2f]", xDim_, yDim_);}
    // methods for obstacles
    void addObstacle(Obstacle obs){Obstacles_.push_back(obs);};
    std::vector<Obstacle> getObstacles() const {return Obstacles_;};
    // methods for agents
    void addAgent(Agent *a){Agents_.push_back(a);};
    std::vector<Agent*> getAgents() const {return Agents_;};
    // printing methods for usability
    void printObstacles()
    {
        OMPL_INFORM("%d Obstacles (xMin, yMin, xMax, yMax): ", Obstacles_.size());
        for (Obstacle o: Obstacles_)
        {
            OMPL_INFORM("   - Obstacle: [%0.2f, %0.2f, %0.2f, %0.2f]", o.xMin_, o.yMin_, o.xMax_, o.yMax_);
        }
    };
    void printAgents()
    {
        OMPL_INFORM("%d Agents: ", Agents_.size());
        for (Agent *a: Agents_)
        {
            OMPL_INFORM("   - Name: %s", a->getName().c_str());
            OMPL_INFORM("     Dynamics: %s", a->getDynamics().c_str());
            OMPL_INFORM("     Start: [%0.2f, %0.2f]", a->getStartLocation()[0], a->getStartLocation()[1]);
            OMPL_INFORM("     Goal: [%0.2f, %0.2f]", a->getGoalLocation()[0], a->getGoalLocation()[1]);
        }
    };
    void printWorld()
    {
        OMPL_INFORM("Map Dimensions: [%0.2f, %0.2f]", getWorldDimensions()[0], getWorldDimensions()[1]);
        printObstacles();
        printAgents();
    }
private:
    double xDim_{0};
    double yDim_{0};
    std::vector<Obstacle> Obstacles_;
    std::vector<Agent*> Agents_;
};

World* yaml2world(std::string file)
{
    YAML::Node config;
    World *w = new World();
    try
    {
        OMPL_INFORM("Path to Problem File: %s", file.c_str());
        config = YAML::LoadFile(file);
        std::cout << "" << std::endl;
        OMPL_INFORM("File loaded successfully. Parsing...");
    }
    catch (const std::exception& e) 
    {
        OMPL_ERROR("Invalid file path. Aborting prematurely to avoid critical error.");
        exit(1);
    }
    try
    {    
        // grab dimensions from problem definition
        const auto& dims = config["Map"]["Dimensions"];
        const int dimx = dims[0].as<double>();
        const int dimy = dims[1].as<double>();
        w->setWorldDimensions(dimx, dimy);
        
        // set Obstacles
        const auto& obs = config["Map"]["Obstacles"];
        for (int i=0; i < obs.size(); i++)
        {
            std::string name = "obstacle" + std::to_string(i);
            Obstacle o = {obs[name][0].as<double>(),    // xMin
                                obs[name][1].as<double>(),      // yMin
                                obs[name][2].as<double>(),      // xMax
                                obs[name][3].as<double>()};     // yMax
            w->addObstacle(o);
        }

        // Define the agents
        const auto& agents = config["Agents"];
        for (int i=0; i < agents.size(); i++)
        {
            std::string name = "agent" + std::to_string(i);
            const std::vector<double> shape{agents[name]["Shape"][0].as<double>(), agents[name]["Shape"][1].as<double>()};
            const std::vector<double> start{agents[name]["Start"][0].as<double>(), agents[name]["Start"][1].as<double>()};
            const std::vector<double> goal{agents[name]["Goal"][0].as<double>(), agents[name]["Goal"][1].as<double>()};
            Agent *a = new Agent(name, agents[name]["Model"].as<std::string>(), shape, start, goal);
            w->addAgent(a);
        }
        OMPL_INFORM("Parsing Complete.");
        std::cout << "" << std::endl;
        w->printWorld();

    }
    catch (const std::exception& e) 
    {
        OMPL_ERROR("Error During Parsing. Aborting prematurely to avoid critical error.");
        exit(1);
    }
    return w;
}