#ifndef _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_LOG_SAVE_FUNCTIONS_HPP_
#define _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_LOG_SAVE_FUNCTIONS_HPP_

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

#include "cvt_functions/quaternion_to_euler.hpp"

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
 * @brief mercury/state ログ保存関数
 * 
 * @param ofs 
 * @param state 
 */
inline void saveMercuryState(std::ofstream &ofs, const RobotStateMsg::SharedPtr &state)
{
    double roll, pitch, yaw;
    double imuRoll, imuPitch, imuYaw;
    char stamp[32];
    std::snprintf(stamp, sizeof(stamp), "%d.%09d",state->header.stamp.sec, state->header.stamp.nanosec);
    
    rs::quaternionToEulerAngle(state->odometry.pose.pose.orientation, &roll, &pitch, &yaw);
    rs::quaternionToEulerAngle(state->odometry.pose.pose.orientation, &imuRoll, &imuPitch, &imuYaw);

    ofs << stamp << ", "
        << state->battery << ", "
        << state->temperature << ", "
        << state->cpu_temperature << ", "
        << state->distance << ", "
        << state->odometry.pose.pose.position.x << ", "
        << state->odometry.pose.pose.position.y << ", "
        << state->odometry.pose.pose.position.z << ", "
        << roll << ", " << pitch << ", " << yaw  << ", "
        << state->odometry.twist.twist.linear.x << ", "
        << state->odometry.twist.twist.linear.y << ", "
        << state->odometry.twist.twist.linear.z << ", "
        << state->odometry.twist.twist.angular.x << ", "
        << state->odometry.twist.twist.angular.y << ", "
        << state->odometry.twist.twist.angular.z << ", "
        << state->current[0] << ", " << state->current[1] << ", " << state->current[2] << ", " << state->current[3] << ", "
        << state->encoder[0] << ", " << state->encoder[1] << ", " << state->encoder[2] << ", " << state->encoder[3] << ", "
        << state->rpm[0] << ", " << state->rpm[1] << ", " << state->rpm[2] << ", " << state->rpm[3] << ", "
        << state->imu.linear_acceleration.x << ", " << state->imu.linear_acceleration.y << ", " << state->imu.linear_acceleration.z << ","
        << state->imu.angular_velocity.x << ", " << state->imu.angular_velocity.y << ", " << state->imu.angular_velocity.z << ","
        << imuRoll << ", " << imuPitch << ", " << imuYaw
        << std::endl;
}

/**
 * @brief PointCloud型ログ保存関数
 * 
 * @param ofs 
 * @param points 
 */
inline void savePointCloud(std::ofstream &ofs, const PointCloud::SharedPtr &points)
{
    char stamp[32];
    std::snprintf(stamp, sizeof(stamp), "%d.%09d",points->header.stamp.sec, points->header.stamp.nanosec);
    ofs << stamp <<std::endl;
    ofs << "x, y, z, r, e" << std::endl;
    for(size_t i=0, size=points->points.size(); i<size; i++){
        ofs << points->points[i].x << ", " << points->points[i].y << ", " << points->points[i].z << ", ";    
        if(!points->channels.empty()) ofs << points->channels[0].values[i] << ", " << points->channels[1].values[i];
        ofs << std::endl;
    }
}

/**
 * @brief Pose型ログ保存関数
 * 
 * @param ofs 
 * @param pose 
 */
inline void savePose(std::ofstream &ofs, const Pose::SharedPtr &pose)
{
    double roll, pitch, yaw;
    rs::quaternionToEulerAngle(pose->orientation, &roll, &pitch, &yaw);
    ofs << pose->position.x << ", " << pose->position.y << ", " << pose->position.z << ", "
        << roll << ", " << pitch << ", " << yaw
        << std::endl;
}

/**
 * @brief Float32MultiArray型ログ保存関数
 * 
 * @param ofs 
 * @param array 
 */
inline void saveFloat32MultiArray(std::ofstream &ofs, const Float32MultiArray::SharedPtr array)
{
    for(size_t i=0, size=array->data.size(); i<size; i++) ofs << array->data[i] << ",";
    ofs << std::endl;
}   
}
#endif

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------