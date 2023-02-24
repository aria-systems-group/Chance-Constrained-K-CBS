#include "utils/common.h"

Location::Location(double x, double y): x_(x), y_(y){};

Obstacle::Obstacle(double x, double y): Location(x, y){}
void Obstacle::printPoints() const
{
    auto exterior_points = boost::geometry::exterior_ring(poly_);
    for (int i=0; i<exterior_points.size(); i++)
    {
        OMPL_INFORM("   - Point(%0.2f, %0.2f)", bg::get<0>(exterior_points[i]), bg::get<1>(exterior_points[i]));
    }
}

const Polygon& Obstacle::getPolygon() const
{
    return poly_;
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

    // assures that the polygon (1) has counter-clockwise points, and (2) is closed
    bg::correct(poly_);
}

Robot::Robot(std::string name, std::string model, Location start, Location goal):
    name_(name), dyn_model_(model), start_(start), goal_(goal) {}
std::string Robot::getName() const {return name_;}
std::string Robot::getDynamicsModel() const {return dyn_model_;}
const Polygon& Robot::getShape() const {return shape_;}
const double Robot::getBoundingRadius() const {return bounding_rad_;};
const Polygon& Robot::getBoundingShape() const {return bounding_poly_;};
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
void Robot::createBoundingShape()
{
    /* Get radius of bounding disk */
    /* First, center robot at origin */
    Polygon r_initial_poly;
    Polygon r_origin_centered;

    const double r_ix = start_.x_;
    const double r_iy = start_.y_;
    bg::correct(r_initial_poly);
    bg::assign(r_initial_poly, shape_);
    bg::strategy::transform::matrix_transformer<double, 2, 2> r_xfrm(
             cos(0), sin(0), (0 - r_ix),
            -sin(0), cos(0), (0 - r_iy),
                      0,          0,  1);
    bg::transform(r_initial_poly, r_origin_centered, r_xfrm);
    bg::correct(r_origin_centered);
    double max_rad = 0.0;
    for (const auto &pt : boost::geometry::exterior_ring(r_origin_centered)) {
        const double x = bg::get<0>(pt);
        const double y = bg::get<1>(pt);
        double curr_rad = sqrt((x*x) + (y*y));
        if (curr_rad > max_rad)
            max_rad = curr_rad;
    }
    bounding_rad_ = max_rad;
    /* Create box around disk */
    // reference point is the center
    Point bott_left(-max_rad, -max_rad);
    Point bott_right(max_rad, -max_rad);
    Point top_right(max_rad, max_rad);
    Point top_left(-max_rad, max_rad);

    bg::append(bounding_poly_.outer(), bott_left);
    bg::append(bounding_poly_.outer(), bott_right);
    bg::append(bounding_poly_.outer(), top_right);
    bg::append(bounding_poly_.outer(), top_left);
    bg::append(bounding_poly_.outer(), bott_left);

    bg::correct(bounding_poly_);

    // for (const auto &pt : boost::geometry::exterior_ring(bounding_poly_)) {
    //     const double x = bg::get<0>(pt);
    //     const double y = bg::get<1>(pt);
    //     std::cout << x << "," << y << std::endl;
    // }
    // std::cout << "done with initialization" << std::endl;
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
    // assures that the polygon (1) has counter-clockwise points, and (2) is closed
    bg::correct(shape_);

    createBoundingShape();
}
