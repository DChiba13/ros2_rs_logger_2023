#ifndef _RROJECT_RYUSEI_LIB_CVT_FUNCTIONS_WORLD_TO_SCREEN_HPP_
#define _RROJECT_RYUSEI_LIB_CVT_FUNCTIONS_WORLD_TO_SCREEN_HPP_

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
 * @brief ワールド座標からカメラ座標への変換
 * 
 * @param wx ワールド x座標
 * @param wy ワールド y座標
 * @param sx スクリーン x座標
 * @param sy スクリーン y座標
 */
inline void worldToScreenPoint(const double &wx, const double &wy, double *sx, double *sy)
{
    *sx = -wy;
    *sy = -wx;
}

/**
 * @brief ワールド座標からカメラ座標への変換
 * 
 * @param wx ワールド x座標
 * @param wy ワールド y座標
 * @param sx スクリーン x座標
 * @param sy スクリーン y座標
 * @param grid_size 1pixelあたりの重み
 * @param offset_x   スクリーン座標系のオフセット
 * @param offset_y   スクリーン座標系のオフセット
 */
inline void worldToScreenPoint(const double &wx, const double &wy, double *sx, double *sy, const double &grid_size, const double &offset_x, const double &offset_y)
{
    *sx = offset_x - wy / grid_size;
    *sy = offset_y - wx / grid_size;
}

}

#endif

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------