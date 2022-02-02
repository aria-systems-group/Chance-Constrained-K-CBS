/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all relevent code for converting the problem 
* file (*.yml) to a World Object 
*********************************************************************/
 
/* Author: Justin Kottinger */

#include <yaml.h>
#include "includes/World.h"


// printing methods for usability
void World::printObstacles()
{
    OMPL_INFORM("%d Obstacle(s) (xMin, yMin, xMax, yMax): ", Obstacles_.size());
    for (Obstacle o: Obstacles_)
    {
        OMPL_INFORM("   - Obstacle: [%0.2f, %0.2f, %0.2f, %0.2f]", o.xMin_, o.yMin_, o.xMax_, o.yMax_);
    }
};
void World::printAgents()
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
void World::printWorld()
{
    OMPL_INFORM("Map Dimensions: [%0.2f, %0.2f]", getWorldDimensions()[0], getWorldDimensions()[1]);
    printObstacles();
    printAgents();
};

// function that parses YAML file to world object
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
        const double dimx = dims[0].as<double>();
        const double dimy = dims[1].as<double>();
        w->setWorldDimensions(dimx, dimy);
        
        // set Obstacles
        const auto& obs = config["Map"]["Obstacles"];
        for (int i=0; i < obs.size(); i++)
        {
            std::string name = "obstacle" + std::to_string(i);
            const double minX = obs[name][0].as<double>();
            const double minY = obs[name][1].as<double>();
            const double maxX = obs[name][2].as<double>();
            const double maxY = obs[name][3].as<double>();            
            // TOP RIGHT VERTEX:
            std::string top_right = std::to_string(maxX) + " " + std::to_string(maxY);
            // TOP LEFT VERTEX:
            std::string top_left = std::to_string(minX) + " " + std::to_string(maxY);
            // BOTTOM LEFT VERTEX:
            std::string bottom_left = std::to_string(minX) + " " + std::to_string(minY);
            // BOTTOM RIGHT VERTEX:
            std::string bottom_right = std::to_string(maxX) + " " + std::to_string(minY);

            // convert to string for easy initializataion
            std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
            polygon poly;
            boost::geometry::read_wkt(points,poly);
            Obstacle o = {minX, minY, maxX, maxY, poly};
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
};