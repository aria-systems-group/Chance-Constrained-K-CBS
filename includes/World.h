/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all relevent code for converting the problem 
* file (*.yml) to a World Object 
*********************************************************************/
 
/* Author: Justin Kottinger */

#pragma once
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <ompl/tools/config/SelfConfig.h>


typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

// rectangular Obstacle defined by two points (xMin, yMin) and (xMax, yMax)
struct Obstacle {
    const double xMin_;
    const double yMin_;
    const double xMax_;
    const double yMax_;
    const polygon poly_;

    void printPoints() const
    {
        auto exterior_points = boost::geometry::exterior_ring(poly_);
        for (int i=0; i<exterior_points.size(); i++)
        {
            std::cout << boost::geometry::get<0>(exterior_points[i]) << " " << boost::geometry::get<1>(exterior_points[i]) <<std::endl;
        }
    }
    polygon::ring_type getPoints() const
    {
        return boost::geometry::exterior_ring(poly_);
    }
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
    void printObstacles();
    void printAgents();
    void printWorld();
private:
    double xDim_{0};
    double yDim_{0};
    std::vector<Obstacle> Obstacles_;
    std::vector<Agent*> Agents_;
};

// function that parses YAML file to world object
World* yaml2world(std::string file);
