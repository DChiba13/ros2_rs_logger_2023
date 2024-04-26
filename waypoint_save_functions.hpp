#ifndef _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_WAYPOINT_SAVE_FUNCTIONS_HPP_
#define _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_WAYPOINT_SAVE_FUNCTIONS_HPP_

//-----------------------------
// include
//-----------------------------
// STL
#include <cmath>
#include <fstream>

// revast msg
#include "ros2_rs_miyauchi_interfaces/msg/way_point.hpp"\

//-----------------------------
// Namespace
//-----------------------------
namespace rs = project_ryusei;
using ros2_rs_miyauchi_interfaces::msg::WayPoint;

//-----------------------------
// Functions
//-----------------------------
namespace project_ryusei{
void saveWayPoint(std::ofstream &ofs, std::vector<WayPoint> &points)
{
    ofs << "mode, position.x, position.y, target_x, target_y, target_yaw, obs_x, obs_y, obs_z" << std::endl;
    double target_yaw;
    for(size_t i=0, size=points.size(); i<size - 1; i++){
        // waypointの目標角を次のwaypointの方向にする
        target_yaw = std::atan2(points[i+1].position.y - points[i].position.y, points[i+1].position.x - points[i].position.x);
        
        // 目標位置を次のwaypoint位置から目標角方向に2m先にする
        if(i != points.size() - 2){
            points[i].target.position.x = 2 * std::cos(target_yaw) + points[i+1].position.x;
            points[i].target.position.y = 2 * std::sin(target_yaw) + points[i+1].position.y;
        }
        // 最後から2つ目の場合には最後の位置を目標位置にする
        else{
            points[i].target.position.x = points[i+1].position.x;
            points[i].target.position.y = points[i+1].position.y;
        }
        ofs << points[i].mode << ", "
            << points[i].position.x << ", " 
            << points[i].position.y << ", " 
            << points[i].target.position.x << ", "
            << points[i].target.position.y << ", "
            << target_yaw << ", "
            << points[i].obstacle_range.x << ", " 
            << points[i].obstacle_range.y << ", "
            << points[i].obstacle_range.z 
            << std::endl;
    }

    // 最後のwaypointは modeを「stop run none」にする．
    // target.postionsはwaypoint座標にする
    // target.yawは1つ前のwaypointのytarget.yawと同じにする．
    ofs << 0x00000000 << ", "
        << points[points.size()-1].position.x << ", "
        << points[points.size()-1].position.y << ", "
        << points[points.size()-1].position.x << ", "
        << points[points.size()-1].position.y << ", "
        << target_yaw << ", "
        << points[points.size()-1].obstacle_range.x << ", "
        << points[points.size()-1].obstacle_range.y << ", "
        << points[points.size()-1].obstacle_range.z 
        << std::endl;
}
}

#endif

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------