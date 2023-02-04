#include "common.h"

Location::Location(double x, double y): x_(x), y_(y){};

Obstacle::Obstacle(double x, double y): Location(x, y){}
void Obstacle::printPoints() const
{
    auto exterior_points = boost::geometry::exterior_ring(poly_);
    for (int i=0; i<exterior_points.size(); i++)
    {
        OMPL_INFORM("   - Point(%0.2f, %0.2f)", boost::geometry::get<0>(exterior_points[i]), boost::geometry::get<1>(exterior_points[i]));
    }
}
Polygon::ring_type Obstacle::getPolyPoints() const
{
    return boost::geometry::exterior_ring(poly_);
}

RectangularObstacle::RectangularObstacle(double x, double y, double len, double width): 
    Obstacle(x, y), len_(len), width_(width) 
{
    // construct the rectangular polygon w/ ref in the center
    Point bott_left(x_ - (len_ / 2), y_ - (width_ / 2));
    Point bott_right(x_ + (len_ / 2), y_ - (width_ / 2));
    Point top_left(x_ - (len_ / 2), y_ + (width_ / 2));
    Point top_right(x_ + (len_ / 2), y_ + (width_ / 2));

    bg::append(poly_.outer(), bott_left);
    bg::append(poly_.outer(), bott_right);
    bg::append(poly_.outer(), top_right);
    bg::append(poly_.outer(), top_left);
    bg::append(poly_.outer(), bott_left);
}

Robot::Robot(std::string name, std::string model, Location start, Location goal):
    name_(name), dyn_model_(model), start_(start), goal_(goal) {}
std::string Robot::getName() const {return name_;}
std::string Robot::getDynamicsModel() const {return dyn_model_;}
Polygon Robot::getShape() const {return shape_;}
Location Robot::getStartLocation() const {return start_;}
Location Robot::getGoalLocation() const {return goal_;}
void Robot::changeDynamics(const std::string newModel) {dyn_model_ = newModel;}
void Robot::printShape() const
{
    auto exterior_points = boost::geometry::exterior_ring(shape_);
    for (int i=0; i<exterior_points.size(); i++) {
        OMPL_INFORM("   - Point(%0.2f, %0.2f)", boost::geometry::get<0>(exterior_points[i]), boost::geometry::get<1>(exterior_points[i]));
    }
}

PointRobot::PointRobot(std::string name, std::string model, Location start, Location goal):
    Robot(name, model, start, goal)
{
    Point p(start.x_, start.y_);
    bg::append(shape_.outer(), p);
}

RectangularRobot::RectangularRobot(std::string name, std::string model, Location start, Location goal, double len, double width):
    Robot(name, model, start, goal), len_(len), width_(width)
{
    // construct a rectangular polygon
    // reference point is the center
    Point bott_left(start.x_ - (len_ / 2), start.y_ - (width_ / 2));
    Point bott_right(start.x_ + (len_ / 2), start.y_ - (width_ / 2));
    Point top_left(start.x_ - (len_ / 2), start.y_ + (width_ / 2));
    Point top_right(start.x_ + (len_ / 2), start.y_ + (width_ / 2));

    bg::append(shape_.outer(), bott_left);
    bg::append(shape_.outer(), bott_right);
    bg::append(shape_.outer(), top_right);
    bg::append(shape_.outer(), top_left);
    bg::append(shape_.outer(), bott_left);
}
