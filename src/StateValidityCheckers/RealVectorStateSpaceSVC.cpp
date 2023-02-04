#include "StateValidityCheckers/RealVectorStateSpaceSVC.h"

RealVectorStateSpaceSVC::RealVectorStateSpaceSVC
    (const oc::SpaceInformationPtr &si, const InstancePtr mrmp_instance, const Robot *r) :
    ob::StateValidityChecker(si), si_(si.get()), mrmp_instance_(mrmp_instance), robot_(r) {}
 
bool RealVectorStateSpaceSVC::isValid(const ob::State *state) const
{
    if (!si_->satisfiesBounds(state))
        return false;
    // Get xy state and theta state (in rad [0, 2*pi]) from state object
    auto compState = state->as<ob::CompoundStateSpace::StateType>();
    auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
    const double cx = xyState->values[0];
    const double cy = xyState->values[1];
    const double theta = compState->as<ob::SO2StateSpace::StateType>(1)->value;

    // beta test: use boost transform instread of manual polygon creation
    Polygon raw;
    Polygon result;
    Polygon initial = robot_->getShape();
    const double ix = robot_->getStartLocation().x_;
    const double iy = robot_->getStartLocation().y_;
    bg::correct(raw);
    bg::assign(raw, initial);
    bg::strategy::transform::matrix_transformer<double, 2, 2> xfrm(
             cos(theta), sin(theta), (cx - ix),
            -sin(theta), cos(theta), (cy - iy),
                      0,          0,  1);
    // std::cout << "raw" << std::endl;
    // for(auto it = boost::begin(boost::geometry::exterior_ring(raw)); it != boost::end(boost::geometry::exterior_ring(raw)); ++it)
    // {
    //     double x = bg::get<0>(*it);
    //     double y = bg::get<1>(*it);
    //     std::cout << x << "," << y << std::endl;
    // }
    bg::transform(raw, result, xfrm);


    // beta test: use boost transform instread of creating a polygon manually
    std::vector<Obstacle*> obs_list = mrmp_instance_->getObstacles();
    for (auto itr = obs_list.begin(); itr != obs_list.end(); itr++) {
        if (! boost::geometry::disjoint(result, (*itr)->getPolyPoints())) {
            // std::cout << "failed" << std::endl;
            // std::cout << cx << std::endl;
            // std::cout << cy << std::endl;
            // std::cout << theta << std::endl;
            // for(auto it = boost::begin(boost::geometry::exterior_ring(result)); it != boost::end(boost::geometry::exterior_ring(result)); ++it)
            // {
            //     double x = bg::get<0>(*it);
            //     double y = bg::get<1>(*it);
            //     std::cout << x << "," << y << std::endl;
            //     //use the coordinates...
            // }
            // (*itr)->printPoints();
            return false;
        }
    }

    // Get important params from car object
    // const double carWidth = robot_->getShape()[0];
    // const double carHeight = robot_->getShape()[1];

    // turn (x,y, theta), width, length to a polygon object
    // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
    // // TOP RIGHT VERTEX:
    // const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
    // const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
    // std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
    // // TOP LEFT VERTEX:
    // const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
    // const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
    // std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
    // // BOTTOM LEFT VERTEX:
    // const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
    // const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
    // std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
    // // BOTTOM RIGHT VERTEX:
    // const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
    // const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
    // std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);

    // // convert to string for easy initializataion
    // std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
    // polygon agent;
    // boost::geometry::read_wkt(points,agent);

    // // create polygon map
    // std::vector<point> map_pts{
    // point(0,0),
    // point(0,w_->getWorldDimensions()[1]),
    // point(w_->getWorldDimensions()[0], w_->getWorldDimensions()[1]),
    // point(w_->getWorldDimensions()[0], 0),
    // point(0,0)};
    // polygon map;
    // boost::geometry::assign_points(map, map_pts);
    // // verify that agent is completely inside another
    // if (! boost::geometry::within(agent, map))
    //     return false;
    return true;
}
