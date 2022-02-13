/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains State validity Checks
* 
* These classes are used by OMPL to plan kinodynamically feasible 
* motion plans.
* 
* Current Capabilities Include:
*       * 2D Collision checking with rectangular obstacles
*       * 3D Collision checking with rectangular obstacles
* 
* Requirements for adding new capabilities:
*       * Function that takes same inputs as existing
*********************************************************************/

/* Author: Justin Kottinger */

#pragma once
#include "World.h"
#include <boost/geometry.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/SpaceInformation.h>


namespace ob = ompl::base;
namespace oc = ompl::control;

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

/********* Definition of 2D State Validity Checker *********/
// this class works for all dynamic models of form x = [x, y, theta, ...]
class isStateValid_2D : public ob::StateValidityChecker
{
public:
    isStateValid_2D(const oc::SpaceInformationPtr &si, const World *w, const Agent *a) :
        ob::StateValidityChecker(si)
        {
            si_ = si.get();
            w_ = w;
            a_ = a;
        }
 
    virtual bool isValid(const ob::State *state) const
    {
        if (!si_->satisfiesBounds(state))
            return false;
        // Get xy state and theta state (in rad [0, 2*pi]) from state object
        auto compState = state->as<ob::CompoundStateSpace::StateType>();
        auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
        const double cx = xyState->values[0];
        const double cy = xyState->values[1];
        const double theta = compState->as<ob::SO2StateSpace::StateType>(1)->value;

        // Get important params from car object
        const double carWidth = a_->getShape()[0];
        const double carHeight = a_->getShape()[1];

        // turn (x,y, theta), width, length to a polygon object
        // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
        // TOP RIGHT VERTEX:
        const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
        const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
        std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
        // TOP LEFT VERTEX:
        const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
        const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
        std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
        // BOTTOM LEFT VERTEX:
        const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
        const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
        std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
        // BOTTOM RIGHT VERTEX:
        const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
        const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
        std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);

        // convert to string for easy initializataion
        std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
        polygon agent;
        boost::geometry::read_wkt(points,agent);
        
        // check agent is disjoint from all obstacles
        for (Obstacle o: w_->getObstacles())
        {
            if (! boost::geometry::disjoint(agent, o.poly_))
                return false;
        }
        // create polygon map
        std::vector<point> map_pts{
        point(0,0),
        point(0,w_->getWorldDimensions()[1]),
        point(w_->getWorldDimensions()[0], w_->getWorldDimensions()[1]),
        point(w_->getWorldDimensions()[0], 0),
        point(0,0)};
        polygon map;
        boost::geometry::assign_points(map, map_pts);
        // verify that agent is completely inside another
        if (! boost::geometry::within(agent, map))
            return false;
        return true;
    }

private:
    const ob::SpaceInformation *si_;
    const World *w_;
    const Agent *a_;
    
};
/********* END 2D State Validity Checker *********/

/********* Definition of 3D State Validity Checker *********/
// this class works for all dynamic models of form x = [x, y, z, theta, ...]
class isStateValid_3D : public ob::StateValidityChecker
{
public:
    isStateValid_3D(const oc::SpaceInformationPtr &si, const World *w, const Agent *a, const std::vector<int> constraints = {}) :
        ob::StateValidityChecker(si)
        {
            si_ = si.get();
            w_ = w;
            a_ = a;
            constraints_ = constraints;
        }
 
    virtual bool isValid(const ob::State *state) const
    {
        if (!si_->satisfiesBounds(state))
            return false;
        OMPL_ERROR("Collision checking not complete!");
        // // Get xy state and theta state (in rad [0, 2*pi]) from state object
        // auto compState = state->as<ob::CompoundStateSpace::StateType>();
        // auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
        // const double cx = xyState->values[0];
        // const double cy = xyState->values[1];
        // const double theta = compState->as<ob::SO2StateSpace::StateType>(1)->value;

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
        // std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "))";
        // polygon agent;
        // boost::geometry::read_wkt(points,agent);

        // // check agent is disjoint from all obstacles
        // for (Obstacle o: w_->getObstacles())
        // {
        //     if (! boost::geometry::disjoint(agent, o.poly_))
        //         return false;
        // }
        // if (constraints_.size() > 0)
        // {
        //     OMPL_ERROR("SUCCESSFULLY ADDED CONSTRAINT");
        // }

        return true;
    }

    // void updateConstraints(std::vector<int> c) {constraints_ = c;};
    
private:
    const ob::SpaceInformation *si_;
    const World *w_;
    const Agent *a_;
    std::vector<int> constraints_;
};

/********* Definition of Composed 2D State Validity Checker *********/
// this class works for all composed dynamic models of form x = [x, y, theta, ...]
class isComposedStateValid_2D : public ob::StateValidityChecker
{
public:
    isComposedStateValid_2D(const oc::SpaceInformationPtr &si, 
        const World *w, const Agent *a, const std::string dynModel) :
        ob::StateValidityChecker(si), si_{si.get()}, w_{w}, a_{a}, dyn_{dynModel}
        {
        }
 
    virtual bool isValid(const ob::State *state) const
    {
        if (!si_->satisfiesBounds(state))
            return false;

        // create polygon map
        std::vector<point> map_pts{
        point(0,0),
        point(0,w_->getWorldDimensions()[1]),
        point(w_->getWorldDimensions()[0], w_->getWorldDimensions()[1]),
        point(w_->getWorldDimensions()[0], 0),
        point(0,0)};
        polygon map;
        boost::geometry::assign_points(map, map_pts);

        // use comp state to fill vehicles vector
        auto compState = state->as<ob::CompoundStateSpace::StateType>();
        std::vector<polygon> vehicles;
        
        int numVs = 0;
        if (dyn_ == "Two Dynamic Cars" )
            numVs = 2;
        else if (dyn_ == "Three Dynamic Cars" )
            numVs = 3;

        
        if (dyn_ == "Two Dynamic Cars" || dyn_ == "Three Dynamic Cars")
        {
            for (int a = 0; a < numVs; a++)
            {
                auto xyState = compState->as<
                    ob::RealVectorStateSpace::StateType>(2*a + 0);
                const double cx = xyState->values[0];
                const double cy = xyState->values[1];
                const double theta = compState->as<
                    ob::SO2StateSpace::StateType>(2*a + 1)->value;

                // Get important params from car object
                const double carWidth = a_->getShape()[0];
                const double carHeight = a_->getShape()[1];

                // turn (x,y, theta), width, length to a polygon object
                // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
                // TOP RIGHT VERTEX:
                const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
                const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
                std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
                // TOP LEFT VERTEX:
                const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
                const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
                std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
                // BOTTOM LEFT VERTEX:
                const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
                const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
                std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
                // BOTTOM RIGHT VERTEX:
                const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
                const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
                std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);

                // convert to string for easy initializataion
                std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
                polygon agent;
                boost::geometry::read_wkt(points,agent);

                vehicles.push_back(agent);
            }
        }
        else
            OMPL_ERROR("Current composed dynamics model is not implemented.");

        if (vehicles.size() != numVs)
        {
            printf("num polys: %lu, num vs: %i \n", vehicles.size(), numVs);
            OMPL_ERROR("Collision Checking not working as expected.");
        }
        
        // check agent is disjoint from all obstacles and map
        for (const auto v: vehicles)
        {
            // check against obs
            for (Obstacle o: w_->getObstacles())
            {
                if (! boost::geometry::disjoint(v, o.poly_))
                {
                    return false;
                }
            }
            // check against map
            if (! boost::geometry::within(v, map))
            {
                return false;
            }
        }
        // check vehicles against each other
        for (int a1 = 0; a1 < vehicles.size(); a1++)
        {
            for (int a2 = 0; a2 < vehicles.size(); a2++)
            {
                if (a1 != a2)
                {
                    if (! boost::geometry::disjoint(vehicles[a1], vehicles[a2]))
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }

private:
    const ob::SpaceInformation *si_;
    const World *w_;
    const Agent *a_;
    const std::string dyn_;
    
};
/********* END 2D Composed State Validity Checker *********/
