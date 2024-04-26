#ifndef _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_LOG_LOAD_FUNCTIONS_HPP_
#define _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_LOG_LOAD_FUNCTIONS_HPP_

//-----------------------------
// include
//-----------------------------
// STL
#include <fstream>

// ROS2
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// Boost
#include <boost/filesystem.hpp>

// revast msg
#include "ros_robocore_interfaces/msg/robot_state_msg.hpp"

#include "cvt_functions/euler_to_quaternion.hpp"

//-----------------------------
// Namespace
//-----------------------------
namespace rs = project_ryusei;
using std_msgs::msg::Float32MultiArray;
using sensor_msgs::msg::PointCloud;
using geometry_msgs::msg::Pose;
using ros_robocore_interfaces::msg::RobotStateMsg;

//-----------------------------
// Functions
//-----------------------------
namespace project_ryusei{
/**
 * @brief mercury/state ログ読込関数
 * 
 * @param ifs 
 * @param state 
 */
inline bool loadMercuryState(std::ifstream &ifs, RobotStateMsg *state)
{
    std::string line;
    
    if(std::getline(ifs, line)){
        double roll, pitch, yaw;
        std::string format = (std::string)
              "%d, %lf, %lf, %lf, "    // time, batt, temp, dis
            + "%lf, %lf, %lf, "        // px, py, pz
            + "%lf, %lf, %lf, "        // ox, oy, oz
            + "%lf, %lf, %lf, "        // vx, vy, vz
            + "%lf, %lf, %lf, "        // wx, wy, wz
            + "%lf, %lf, %lf, %lf,"    // i1, i2, i3, i4
            + "%d, %d, %d, %d,"        // enc1, enc2, enc3, enc4,
            + "%d, %d, %d, %d,"        // rpm1, rpm2, rpm3, rpm4
            + "%lf, %lf, %lf, "        // imu- ax, ay, az
            + "%lf, %lf, %lf, "        // imu- wx, wy, wz
            + "%lf, %lf, %lf, ";       // imu- ox, oy, oz

        std::sscanf(line.c_str(), format.c_str(),
            &state->header.stamp.sec, &state->battery, &state->temperature, &state->distance,
            &state->odometry.pose.pose.position.x, &state->odometry.pose.pose.position.y, &state->odometry.pose.pose.position.z,
            &roll, &pitch, &yaw,
            &state->odometry.twist.twist.linear.x, &state->odometry.twist.twist.linear.y, &state->odometry.twist.twist.linear.z, 
            &state->odometry.twist.twist.angular.x, &state->odometry.twist.twist.angular.y, &state->odometry.twist.twist.angular.z, 
            &state->current[0], &state->current[1], &state->current[2], &state->current[3], 
            &state->encoder[0], &state->encoder[1], &state->encoder[2], &state->encoder[3], 
            &state->rpm[0], &state->rpm[1], &state->rpm[2], &state->rpm[3], 
            &state->imu.linear_acceleration.x, &state->imu.linear_acceleration.y, &state->imu.linear_acceleration.z, 
            &state->imu.angular_velocity.x, &state->imu.angular_velocity.y, &state->imu.angular_velocity.z, 
            &roll, &pitch, &yaw
        );
        rs::eulerToQuaternionAngle(roll, pitch, yaw, &state->odometry.pose.pose.orientation);
        state->imu.orientation = state->odometry.pose.pose.orientation;
    }
    else{
        return false;
    }
    return true;
}

/**
 * @brief PointCloud型ログ読込関数
 * 
 * @param ifs 
 * @param points 
 */
inline void loadPointCloud(std::ifstream &ifs, PointCloud *points)
{
    // header 読み飛ばし
    std::string line;
    std::getline(ifs, line);
    std::getline(ifs, line);

    points->points.clear();
    points->channels.clear();
    points->channels.resize(2);

    while(std::getline(ifs, line)){
        geometry_msgs::msg::Point32 p;
        float d, e;
        std::sscanf(line.c_str(), "%f, %f, %f, %f, %f", &p.x, &p.y, &p.z, &d, &e);
        points->points.push_back(p);
        points->channels[0].values.push_back(d);
        points->channels[1].values.push_back(e);
    }
}

/**
 * @brief Float32MultiArray型ログ読込関数
 * 
 * @param ifs 
 * @param array 
 */
inline bool loadFloat32MultiArray(std::ifstream &ifs, Float32MultiArray *array)
{
    std::string line;

    if(std::getline(ifs, line)){
        std::sscanf(line.c_str(), 
                    "%f, %f, %f, %f, %f, %f, %f",
                    &array->data[0],
                    &array->data[1], 
                    &array->data[2], 
                    &array->data[3], 
                    &array->data[4], 
                    &array->data[5],
                    &array->data[6]);
    }
    else{
        return false;
    }
    return true;
}   
}

#endif // _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_LOG_LOAD_FUNCTIONS_HPP_

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------