#ifndef _RS_COMMON_UTILITIES_HPP_
#define _RS_COMMON_UTILITIES_HPP_

#include "defs.hpp"

namespace project_ryusei
{
bool loadOdometryFromLog(const std::string &path, Odometry &odometry);
bool loadPoseFromLog(const std::string &path, Pose3D &pose);
bool loadPointsFromLog(const std::string &path, std::vector<cv::Point3f> &points);
bool loadPointsFromLog(const std::string &path, std::vector<LidarData> &points);
bool loadLocalMap(const std::string &pose_path, const std::string &points_path, LocalMap &map);
bool writeOdometry(const std::string &path, const Odometry &state);
bool writePose(const std::string &path, const Pose3D &pose);
//rosで高速に送信するための関数
bool writePointCloud(const std::string &path, const cv::Point3f *points, const std::vector<float> &intensity);
//通常使用
bool writePointCloud(const std::string &path, const std::vector<LidarData> &points);
bool writePointCloudBinary(const std::string &path, cv::Point3f *points, std::vector<float> &intensity);
bool writePointCloudBinary(const std::string &path, std::vector<LidarData> &points);

}
#endif