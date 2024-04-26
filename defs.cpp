#include "defs.hpp"

using namespace std;

const double RAD_TO_DEG = 180.0 / M_PI;
const double DEG_TO_RAD = M_PI / 180.0;

namespace project_ryusei
{

// #region Pose3D
Pose3D::Pose3D()
{

}

Pose3D::Pose3D(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
  : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw)
{
  
}

std::ostream& operator << (std::ostream &os, const Pose3D &p)
{
  os << "[" << p.x << " , " << p.y << " , " << p.z << " , " << p.roll * RAD_TO_DEG << " , " << p.pitch * RAD_TO_DEG << " , " << p.yaw * RAD_TO_DEG;
  return os;
}
// #endregion Pose3D

// #region Velocity3D
Velocity3D::Velocity3D()
{

}

Velocity3D::Velocity3D(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
  : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw)
{

}

std::ostream& operator << (std::ostream &os, const Velocity3D &v)
{
  os << "[" << v.x << " , " << v.y << " , " << v.z << " , " << v.roll * RAD_TO_DEG << " , " << v.pitch * RAD_TO_DEG << " , " << v.yaw * RAD_TO_DEG;
  return os;
}

// #endregion Velocity3D

// #region LocalMap
LocalMap::LocalMap()
{

}

std::ostream& operator << (std::ostream &os, const LocalMap &lm)
{
  os << "[" << lm.pose.x << " , " << lm.pose.y << " , " << lm.pose.yaw * RAD_TO_DEG << "]" << " : " << lm.points.size();
  return os;
}
// #endregion LocalMap

// #region ScanMatchingScore
// ScanMatchingScore::ScanMatchingScore()
// {

// }

// ScanMatchingScore::ScanMatchingScore(double _raw, double _ref, double _que)
//   : raw(_raw), ref(_ref), que(_que)
// {

// }

// std::ostream& operator << (std::ostream &os, const ScanMatchingScore &p)
// {
//   os << "[" << p.ref << " , " << p.que << " , " << p.raw << "]";
//   return os;
// }
// #endregion ScanMatchingScore

// #region SignalInfo
// SignalInfo::SignalInfo()
// {

// }

// SignalInfo::SignalInfo(SignalColor _color, const cv::Point &_pos) 
//   : color(_color), pos(_pos)
// {

// }

// std::ostream& operator << (std::ostream &os, const SignalInfo &info)
// {
//   if(info.color == SignalColor::Red) os << "Red" << " : " << info.pos;
//   else os << "Green" << " : " << info.pos;
//   return os;
// }
// #endregion SignalInfo

// #region LidarData
LidarData::LidarData()
{

}

// #endregion LidarData

// #region Obstacle
// Obstacle::Obstacle()
// {

// }

// double Obstacle::getSize() const
// {
//   double sz;
//   width * height > width * depth ? sz = width * height : sz = width * depth;
//   return sz;
// };

// cv::Point3f Obstacle::getAbsoluteCenter() const
// {
//   cv::Point3f c(.0f, .0f, .0f);
//   int sz = absolute_points.size();
//   if(sz <= 0) return c;
//   for(auto &pt : absolute_points){
//     c.x += pt.x;
//     c.y += pt.y;
//     c.z += pt.z;
//   }
//   c.x /= sz;
//   c.y /= sz;
//   c.z /= sz;
//   return c;
// };

// cv::Point3f Obstacle::getRelativeCenter() const
// {
//   cv::Point3f c(.0f, .0f, .0f);
//   int sz = relative_points.size();
//   if(sz <= 0) return c;
//   for(auto &pt : relative_points){
//     c.x += pt.x;
//     c.y += pt.y;
//     c.z += pt.z;
//   }
//   c.x /= sz;
//   c.y /= sz;
//   c.z /= sz;
//   return c;
// }

// #endregion Obstacle

// #region ObstacleFeature
// ObstacleFeature::ObstacleFeature()
// {

// }
// #endregion ObstacleFeature

// #region TrackableObstacle
// TrackableObstacle::TrackableObstacle()
// {

// }
// #endregion TrackableObstacle

// #region WayPoint
// WayPoint::WayPoint()
// {
  
// }
// #endregion WayPoint

// #region Vector3D
Vector3D::Vector3D()
{

}
// #endregion Vector3D

// #region Odometry
Odometry::Odometry()
{

}
// #endregion Odometry

}