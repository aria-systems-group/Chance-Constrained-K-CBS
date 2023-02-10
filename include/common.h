#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <ompl/util/Console.h>



namespace bg = boost::geometry;


typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::polygon<Point, false, true> Polygon;


class Location {
public:
    Location(double x, double y);
    const double x_;
    const double y_;
};

// An Obstacle is a polygon located at a reference Location
class Obstacle: public Location {
public:
    Obstacle(double x, double y);
    void printPoints() const;
    Polygon getPolygon() const;
    Polygon::ring_type getPolyPoints() const;
protected:
    Polygon poly_;
};

// a rectangular obstacle class
class RectangularObstacle: public Obstacle {
public:
    RectangularObstacle(double x, double y, double len, double width);
private:
    const double len_;
    const double width_;
};

// A Robot has a name, shape, dynamics, start Location and goal Location
class Robot
{
public:
    Robot(std::string name, std::string model, Location start, Location goal);
    std::string getName() const;
    std::string getDynamicsModel() const;
    Polygon getShape() const;
    Location getStartLocation() const;
    Location getGoalLocation() const;
    void changeDynamics(const std::string newModel);
    void printShape() const;
protected:
    std::string name_;
    std::string dyn_model_;
    Polygon shape_;
    Location start_;
    Location goal_;
};

class PointRobot: public Robot {
public:
    PointRobot(std::string name, std::string model, Location start, Location goal);
};

// a rectangular robot class
class RectangularRobot: public Robot {
public:
    RectangularRobot(std::string name, std::string model, 
        Location start, Location goal, double len, double width);
private:
    const double len_;
    const double width_;
};
