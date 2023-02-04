#include "StateValidityCheckers/MultiRobotStateSpaceSVC.h"

MultiRobotStateSpaceSVC::MultiRobotStateSpaceSVC
    (const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, std::string dyn) :
    ob::StateValidityChecker(si), si_(si.get()), mrmp_instance_(mrmp_instance), robot_(r), dyn_(dyn) {}
 
bool MultiRobotStateSpaceSVC::isValid(const ob::State *state) const
{
    if (!si_->satisfiesBounds(state))
        return false;
    // // create polygon map
    // std::vector<point> map_pts{
    // Point(0,0),
    // Point(0,w_->getWorldDimensions()[1]),
    // Point(w_->getWorldDimensions()[0], w_->getWorldDimensions()[1]),
    // Point(w_->getWorldDimensions()[0], 0),
    // Point(0,0)};
    // Polygon map;
    // boost::geometry::assign_points(map, map_pts);

    // use comp state to fill vehicles vector
    auto compState = state->as<ob::CompoundStateSpace::StateType>();
    std::vector<Polygon> vehicles;
        
    int numVs = 0;
    if (dyn_ == "Two Dynamic Cars" )
        numVs = 2;
    else if (dyn_ == "Three Dynamic Cars" )
        numVs = 3;
        
    if (dyn_ == "Two Dynamic Cars" || dyn_ == "Three Dynamic Cars")
    {
        for (int i = 0; i < numVs; i++)
        {
            auto xyState = compState->as<
                ob::RealVectorStateSpace::StateType>(2*i + 0);
            const double cx = xyState->values[0];
            const double cy = xyState->values[1];
            const double theta = compState->as<ob::SO2StateSpace::StateType>(2*i + 1)->value;

            // // Get important params from car object
            // const double carWidth = a_->getShape()[0];
            // const double carHeight = a_->getShape()[1];

            // // turn (x,y, theta), width, length to a polygon object
            // // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
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
            // Polygon agent;
            // boost::geometry::read_wkt(points,agent);
            
            // beta test: use boost transform instread of creating a polygon manually
            Polygon raw;
            Polygon result;
            Polygon initial = robot_->getShape();
            bg::correct(raw);
            bg::assign(raw, initial);
            bg::strategy::transform::matrix_transformer<double, 2, 2> xfrm(
                     cos(theta), sin(theta), cx,
                    -sin(theta), cos(theta), cy,
                      0,          0,  1);

            bg::transform(raw, result, xfrm);
            vehicles.push_back(result);
        }
    }
    else
        OMPL_ERROR("Current composed dynamics model is not implemented.");

    if (vehicles.size() != numVs) {
        printf("num polys: %lu, num vs: %i \n", vehicles.size(), numVs);
        OMPL_ERROR("Collision Checking not working as expected.");
    }
        
    // check agent is disjoint from all obstacles and map
    std::vector<Obstacle*> obs_list = mrmp_instance_->getObstacles();
    for (const auto v: vehicles) {
        // check against obs
        for (auto itr = obs_list.begin(); itr != obs_list.end(); itr++) {
            if (! boost::geometry::disjoint(v, (*itr)->getPolyPoints()))
            return false;
        }   
        // // check against map
        // if (! boost::geometry::within(v, map))
        //         return false;
    }
    // check vehicles against each other
    for (int a1 = 0; a1 < vehicles.size(); a1++) {
        for (int a2 = 0; a2 < vehicles.size(); a2++) {
            if (a1 != a2) {
                if (! boost::geometry::disjoint(vehicles[a1], vehicles[a2]))
                    return false;
            }
        }
    }
    return true;
}
