#ifndef _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_WAYPOINT_LOAD_FUNCTIONS_HPP_
#define _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_WAYPOINT_LOAD_FUNCTIONS_HPP_

//-----------------------------
// include
//-----------------------------
// STL
#include <vector>
#include <string>
#include <fstream>

// revast msg
#include "ros2_rs_miyauchi_interfaces/msg/way_point.hpp"
#include "cvt_functions/euler_to_quaternion.hpp"

//-----------------------------
// Namespace
//-----------------------------
namespace rs = project_ryusei;
using ros2_rs_miyauchi_interfaces::msg::WayPoint;

//-----------------------------
// Functions
//-----------------------------
namespace project_ryusei{
void loadWayPoint(std::ifstream &ifs, std::vector<WayPoint> *points)
{
    if(!ifs) return;

    points->clear();
    std::string line;
    std::getline(ifs, line);
    while(std::getline(ifs, line)){
        WayPoint wpt;
        double target_yaw;
        std::sscanf(line.c_str(), "%d, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
            &wpt.mode,
            &wpt.position.x, &wpt.position.y,
            &wpt.target.position.x, &wpt.target.position.y, &target_yaw,
            &wpt.obstacle_range.x, &wpt.obstacle_range.y, &wpt.obstacle_range.z
        );

        rs::eulerToQuaternionAngle(0, 0, target_yaw, &wpt.target.orientation);
        points->push_back(wpt);
    }
    std::cout << "debug3" << std::endl;
}
}

#endif

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------