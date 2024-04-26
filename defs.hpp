#ifndef _RS_COMMON_2023_DEFS_HPP_
#define _RS_COMMON_2023_DEFS_HPP_

#include <opencv2/opencv.hpp>

namespace project_ryusei
{

/*** 信号の色を表す型 ***/
// enum class SignalColor{
//   Red = 1,
//   Green = 2
// };

// template<class T> class NumericRange {
// public:
//   NumericRange(){};
//   NumericRange(T _min, T _max){ 
//     min = _min;
//     max = _max;
//   }
//   friend std::ostream& operator << (std::ostream &os, const NumericRange<T> &p){
//     os << "[" << p.min << " , " << p.max << "]" << std::endl;
//     return os;
//   }
//   T min = 0, max = 0;
// };

/*** ロボット等の位置情報(x, y, z, roll, pitch, yaw)を表す型 ***/
class Pose3D{
public:
  Pose3D();
  Pose3D(double x, double y, double z, double roll, double pitch, double yaw);
  friend std::ostream& operator << (std::ostream &os, const Pose3D &p);
  double x = .0, y = .0, z = .0, roll = .0, pitch = .0, yaw = .0;
};

/*** ロボット等の速度を表す型 ***/
class Velocity3D{
public:
  Velocity3D();
  Velocity3D(double x, double y, double z, double roll, double pitch, double yaw);
  friend std::ostream& operator << (std::ostream &os, const Velocity3D &v);
  double x = .0, y = .0, z = .0, roll = .0, pitch = .0, yaw = .0;
};

/*** 位置情報と点群から構成される局所的環境地図を表す型 ***/
class LocalMap
{
public:
  LocalMap();
  friend std::ostream& operator << (std::ostream &os, const LocalMap &lm);
  /*** 点群 ***/
  std::vector<cv::Point3f> points;
  /*** ロボット位置 ***/
  Pose3D pose;
};

/*** マッチング結果に対するスコア ***/
// class ScanMatchingScore
// {
// public:
//   ScanMatchingScore();
//   ScanMatchingScore(double raw, double ref, double que);
//   friend std::ostream& operator << (std::ostream &os, const ScanMatchingScore &p);
//   double ref = .0, que = .0, raw = .0;
// };

/*** 信号の画像上の位置と色を表す型 ***/
// class SignalInfo{
// public:
//   SignalInfo();
//   SignalInfo(SignalColor color, const cv::Point &pos);
//   friend std::ostream& operator << (std::ostream &os, const SignalInfo &info);
//   SignalColor color = SignalColor::Red;
//   cv::Point pos;
// };

/*** LiDARから取得する点群データの型 ***/
class LidarData{
public:
  LidarData();
  float x = .0f, y = .0f, z = .0f, range = .0f, reflectivity = .0f;
};

// class Obstacle{
// public:
//   Obstacle();
//   double getSize() const;
//   cv::Point3f getAbsoluteCenter() const;
//   cv::Point3f getRelativeCenter() const;
//   std::vector<cv::Point3f> relative_points, absolute_points;
//   std::vector<LidarData> points;
//   double width, height, depth;
//   double relative_velocity_x, relative_velocity_y, relative_velocity_z;
//   double absolute_velocity_x, absolute_velocity_y, absolute_velocity_z;
//   int flag;
// };

// class ObstacleFeature{
// public:
//   ObstacleFeature();
//   std::vector<float> hog;
//   std::vector<float> reflectivity;
//   float width, height;
//   cv::Point3f center;
// };

// class TrackableObstacle{
// public:
//   TrackableObstacle();
//   std::vector<Obstacle> obstacles;
//   std::vector<ObstacleFeature> features;
//   std::vector<cv::Rect> rectangles;
//   std::vector<int> pairs;
//   cv::Mat lidar_img;
//   cv::Mat clusters;
//   Pose3D pose;
//   int size;
// };

// class WayPoint{
// public:
//   WayPoint();
//   Pose3D goal;
// };

class Vector3D{
public:
  Vector3D();
  double x = .0, y = .0, z = .0;
};

class Odometry{
public:
  Odometry();
  Pose3D pose;
  Velocity3D velocity;
  Vector3D acceleration;
  std::vector<int> encoder, rpm, pwm;
  double distance = .0;
  double battery = .0;
  std::vector<double> current;
  double temperature = .0, cpu_temperature = .0;
  bool is_emergency = false;
};

}

#endif