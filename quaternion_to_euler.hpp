#ifndef _RROJECT_RYUSEI_LIB_CVT_FUNCTIONS_QUATERNION_TO_EULER_HPP_
#define _RROJECT_RYUSEI_LIB_CVT_FUNCTIONS_QUATERNION_TO_EULER_HPP_

//-----------------------------
// include
//-----------------------------
// ROS2
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//-----------------------------
// Namespace
//-----------------------------

//-----------------------------
// Functions
//-----------------------------
namespace project_ryusei{
/**
 * @brief Quaternion角からEuler角への変換関数
 * 
 * @param q quaternion角 
 * @param roll
 * @param pitch 
 * @param yaw 
 */
inline void quaternionToEulerAngle(const geometry_msgs::msg::Quaternion &q, double *roll, double *pitch, double *yaw)
{
    tf2::Matrix3x3(tf2::Quaternion(q.x,q.y,q.z, q.w)).getRPY(*roll, *pitch, *yaw);
}
}

#endif

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------