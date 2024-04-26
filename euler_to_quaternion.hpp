#ifndef _RROJECT_RYUSEI_LIB_CVT_FUNCTIONS_EULER_TO_QUATERNION_HPP_
#define _RROJECT_RYUSEI_LIB_CVT_FUNCTIONS_EULER_TO_QUATERNION_HPP_

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
 * @brief Euler角からQuaternion角への変換関数
 *  
 * @param roll
 * @param pitch 
 * @param yaw 
 * @param q quaternion角
 */
inline void eulerToQuaternionAngle(const double &roll, const double &pitch, const double &yaw, geometry_msgs::msg::Quaternion *q)
{
    tf2::Quaternion tf2q;
    tf2q.setRPY(roll, pitch, yaw);
    q->w = tf2q.getW(); q->x = tf2q.getX(); q->y = tf2q.getY(); q->z = tf2q.getZ(); 
}
}

#endif

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------