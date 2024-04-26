#include "logger_lib.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <experimental/filesystem>
#include <stdio.h>
#include <fstream>

using namespace std;
namespace pr = project_ryusei;
namespace fs = std::experimental::filesystem;
namespace prop = boost::property_tree;

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)



bool pr::loadOdometryFromLog(const string &path, Odometry &odometry)
{
  /*** Check file format ***/
  fs::path p(path);
  if(p.extension() != ".json"){
    cerr << path << " is invalid format" << endl;
    return false;
  }
  /*** Open json file ***/
  prop::ptree pt;
  try{
    read_json(path, pt);
    odometry.current.clear();
    odometry.encoder.clear();
    odometry.rpm.clear();
    if(auto v = pt.get_optional<double>("Pose.X")) odometry.pose.x = v.get();
    if(auto v = pt.get_optional<double>("Pose.Y")) odometry.pose.y = v.get();
    if(auto v = pt.get_optional<double>("Pose.Z")) odometry.pose.z = v.get();
    if(auto v = pt.get_optional<double>("Pose.Roll")) odometry.pose.roll = DEG_TO_RAD * v.get();
    if(auto v = pt.get_optional<double>("Pose.Pitch")) odometry.pose.pitch = DEG_TO_RAD * v.get();
    if(auto v = pt.get_optional<double>("Pose.Yaw")) odometry.pose.yaw = DEG_TO_RAD * v.get();
    if(auto v = pt.get_optional<double>("Velocity.X")) odometry.velocity.x = v.get();
    if(auto v = pt.get_optional<double>("Velocity.Y")) odometry.velocity.y = v.get();
    if(auto v = pt.get_optional<double>("Velocity.Z")) odometry.velocity.z = v.get();
    if(auto v = pt.get_optional<double>("Velocity.Roll")) odometry.velocity.roll = DEG_TO_RAD * v.get();
    if(auto v = pt.get_optional<double>("Velocity.Pitch")) odometry.velocity.pitch = DEG_TO_RAD * v.get();
    if(auto v = pt.get_optional<double>("Velocity.Yaw")) odometry.velocity.yaw = DEG_TO_RAD * v.get();
    if(auto v = pt.get_optional<double>("Battery")) odometry.battery = v.get();
    if(auto v = pt.get_optional<double>("Distance")) odometry.distance = v.get();
    if(auto v = pt.get_optional<double>("Temperature.Internal")) odometry.temperature = v.get();
    if(auto v = pt.get_optional<double>("Temperature.CPU")) odometry.cpu_temperature = v.get();
    BOOST_FOREACH(prop::ptree::value_type& child, pt.get_child("Current")){
      const prop::ptree& pt_child = child.second;
      auto val = pt_child.get_optional<double>("");
      odometry.current.push_back(val.get());
    }
    BOOST_FOREACH(prop::ptree::value_type& child, pt.get_child("Encoder")){
      const prop::ptree& pt_child = child.second;
      auto val = pt_child.get_optional<int>("");
      odometry.encoder.push_back(val.get());
    } 
    BOOST_FOREACH(prop::ptree::value_type& child, pt.get_child("RPM")){
      const prop::ptree& pt_child = child.second;
      auto val = pt_child.get_optional<int>("");
      odometry.rpm.push_back(val.get());
    }
    try{
      BOOST_FOREACH(prop::ptree::value_type& child, pt.get_child("PWM")){
        const prop::ptree& pt_child = child.second;
        auto val = pt_child.get_optional<int>("");
        odometry.pwm.push_back(val.get());
      }
    } catch(...){
      odometry.pwm.resize(odometry.rpm.size());
    }
  } catch(const prop::ptree_error &e){
    cerr << "Failed to load " << path << endl;
    cerr << "Error message : " << e.what() << endl;
    return false;
  }
  return true;
}

bool pr::loadPoseFromLog(const string &path, Pose3D &pose)
{
  /*** Check file format ***/
fs::path p(path);
if(p.extension() != ".json"){
  cerr << path << " is invalid format" << endl;
  return false;
}
/*** Open json file ***/
prop::ptree pt;
try{
  read_json(path, pt);
  if(auto v = pt.get_optional<double>("Pose.X")) pose.x = v.get();
  if(auto v = pt.get_optional<double>("Pose.Y")) pose.y = v.get();
  if(auto v = pt.get_optional<double>("Pose.Z")) pose.z = v.get();
  if(auto v = pt.get_optional<double>("Pose.Roll")) pose.roll = DEG_TO_RAD * v.get();
  if(auto v = pt.get_optional<double>("Pose.Pitch")) pose.pitch = DEG_TO_RAD * v.get();
  if(auto v = pt.get_optional<double>("Pose.Yaw")) pose.yaw = DEG_TO_RAD * v.get();

} catch(const prop::ptree_error &e){
  cerr << "Failed to load " << path << endl;
  cerr << "Error message : " << e.what() << endl;
  return false;
}
  return true;
}

// csv pcd pcdbainari
//binaryファイルオープン時に開けなかった場合のエラー分を書く
bool pr::loadPointsFromLog(const string &path, std::vector<cv::Point3f> &points)
{
  string read_str, separator;
  ifstream points_file(path);
  fs::path p(path);
  vector<string> split_array;
  //-----エラー処理-----
  if(points_file.fail()){
    cerr << "Failed to open points file " << path << endl;
    return false;
  }
  //--------------------
  //-----csvファイル読み込み-----
  if(p.extension() == ".csv"){
    cout << "read .csv file" << endl;
    separator = ",";
    //-----２行読み飛ばし-----
    getline(points_file, read_str);
    getline(points_file, read_str);
    //------------------------
    while(getline(points_file, read_str)){
      boost::algorithm::split(split_array, read_str, boost::is_any_of(separator));
      points.push_back(cv::Point3f(stof(split_array[0]), stof(split_array[1]), stof(split_array[2])));
    }
  } 
  //-----------------------------
  //-----pcdファイル読み込み-----
  if(p.extension() == ".pcd"){
    cout << "read .pcd file" << endl;
    separator = " "; 
    int size_line = 8;
    int read_line_cnt = 0;
    int points_size;
    string file_type;
    vector<string> points_size_split_array;
    //-----points file size 取得-----
    while(read_line_cnt++ < size_line) getline(points_file, read_str); //
    boost::algorithm::split(points_size_split_array, read_str, boost::is_any_of(separator));
    points_size = stoi(points_size_split_array[1]);
    //-------------------------------
    points.resize(points_size);
    std::vector<float> intensity(points_size);
    //------file type 取得------
    getline(points_file, read_str);
    file_type = read_str;
    //--------------------------
    //------asciiファイルの場合の処理-----
    if(file_type == "DATA ascii"){
      cout << "read ascii file: " << path << endl;
      for(int i = 0; i < points_size; i++){
        getline(points_file, read_str);
        boost::algorithm::split(split_array, read_str, boost::is_any_of(separator));
        points[i] = cv::Point3f(stof(split_array[0]), stof(split_array[1]), stof(split_array[2]));
      }
    }
    //------------------------------------
    //------binaryファイルの場合の処理-----
    if(file_type == "DATA binary"){
      cout << "read binary file :" << path << endl;
      points_file.close();
      FILE *fp;
      char read_str[50];
      fp = fopen(path.c_str(), "rb");
      //-----エラー処理-----
      if(fp == NULL){
        cerr << "Failed to open points file binary: " << path << endl;
        return false;
      }
      //--------------------
      for(int i = 0; i < 9; i++) fgets(read_str ,50, fp); //文字列部分の読み飛ばし
      for(int i = 0; i < points_size; i++){
        fread(&points[i], sizeof(float) * 3, 1, fp);
        fread(&intensity[i], sizeof(float), 1, fp);
      }
    }
    //-------------------------------------
  } 
  return true;
}

bool pr::loadPointsFromLog(const std::string &path, std::vector<LidarData> &points)
{
  string read_str, separator;
  ifstream points_file(path);
  fs::path p(path);
  vector<string> split_array;
  //-----エラー処理-----
  if(points_file.fail()){
    cerr << "Failed to open points file " << path << endl;
    return false;
  }
  //--------------------
  //-----csvファイル読み込み-----
  if(p.extension() == ".csv") {
    cout << "read .csv file" << endl;
    separator = ",";
    //-----２行読み飛ばし-----
    getline(points_file, read_str);
    getline(points_file, read_str);
    //------------------------
    while(getline(points_file, read_str)){
      boost::algorithm::split(split_array, read_str, boost::is_any_of(separator));
      LidarData data;
      data.x = stof(split_array[0]);
      data.y = stof(split_array[1]);
      data.z = stof(split_array[2]);
      data.range = sqrt(data.x * data.x + 
                        data.y * data.y +
                        data.z * data.z);
      data.reflectivity = stof(split_array[4]);
      points.push_back(data);
    }
  }
  //------------------------------
  //-----pcdファイル読み込み-----
  if(p.extension() == ".pcd") {
    separator = " ";
    int size_line = 8;
    int read_line_cnt = 0;
    int points_size;
    string file_type;
    vector<string> points_size_split_array;
    //-----points file size 取得-----
    while(read_line_cnt++ < size_line) getline(points_file, read_str);
    boost::algorithm::split(points_size_split_array, read_str, boost::is_any_of(separator));
    points_size = stoi(points_size_split_array[1]);
    //-------------------------------
    points.resize(points_size);
    //-----file type 取得-----
    getline(points_file, read_str);
    file_type = read_str;
    //------------------------
    //-----asciiファイルの場合の処理-----
    if(file_type == "DATA ascii"){
      cout << "read ascii file: " << path << endl;
      for(int i = 0; i < points_size; i++){
        getline(points_file, read_str);
        cout << read_str << endl;
        boost::algorithm::split(split_array, read_str, boost::is_any_of(separator));
        cout << split_array[3] << endl;
        points[i].x = stof(split_array[0]);
        points[i].y = stof(split_array[1]);
        points[i].z = stof(split_array[2]);
        points[i].range = sqrt(points[i].x * points[i].x + 
                              points[i].y * points[i].y +
                              points[i].z * points[i].z);
        points[i].reflectivity = stof(split_array[3]);
      }
    }
    //-----------------------------------
    //-----binaryファイルの場合の処理-----
    if(file_type == "DATA binary"){
      cout << "read binary file: " << path << endl;
      points_file.close();
      FILE *fp;
      char read_str[50];
      fp = fopen(path.c_str(), "rb");
      //-----エラー処理-----
      if(fp == NULL){
        cerr << "Failed to open points file binary: " << path << endl;
        return false;
      }
      //--------------------
      for(int i = 0; i < 9; i++) fgets(read_str, 50, fp); //文字列部分の読み飛ばし
      for(int i = 0; i < points_size; i++){
        fread(&points[i].x, sizeof(float), 1, fp);
        fread(&points[i].y, sizeof(float), 1, fp);
        fread(&points[i].z, sizeof(float), 1, fp);
        points[i].range = sqrt(points[i].x * points[i].x + 
                              points[i].y * points[i].y +
                              points[i].z * points[i].z);
        fread(&points[i].reflectivity, sizeof(float), 1, fp);
      }
    }
    //--------------------------------
  }
  return true;
}


bool pr::loadLocalMap(const string &pose_path, const std::string &points_path, LocalMap &map)
{
  Pose3D pose;
  vector<cv::Point3f> points;
  pr::loadPoseFromLog(pose_path, pose);
  pr::loadPointsFromLog(points_path, points);
  map.points.resize(points.size());
  map.pose = pose;
  map.points = points;
  return true;
}

bool pr::writeOdometry(const std::string &path, const Odometry &odometry)
{
  /*** Declare property tree ***/
  prop::ptree pt, pt_child;
  /*** Write down robot pose ***/
  pt.put("Pose.X", odometry.pose.x);
  pt.put("Pose.Y", odometry.pose.y);
  pt.put("Pose.Z", odometry.pose.z);
  pt.put("Pose.Roll", RAD_TO_DEG * odometry.pose.roll);
  pt.put("Pose.Pitch", RAD_TO_DEG * odometry.pose.pitch);
  pt.put("Pose.Yaw", RAD_TO_DEG * odometry.pose.yaw);
  /*** Write down velocity ***/
  pt.put("Velocity.X", odometry.velocity.x);
  pt.put("Velocity.Y", odometry.velocity.y);
  pt.put("Velocity.Z", odometry.velocity.z);
  pt.put("Velocity.Roll", RAD_TO_DEG * odometry.velocity.roll);
  pt.put("Velocity.Pitch", RAD_TO_DEG * odometry.velocity.pitch);
  pt.put("Velocity.Yaw", RAD_TO_DEG * odometry.velocity.yaw);
  /*** Write down acceleration ***/
  pt.put("Acceleration.X", odometry.acceleration.x);
  pt.put("Acceleration.Y", odometry.acceleration.y);
  pt.put("Acceleration.Z", odometry.acceleration.z);
  /*** Write down battery ***/
  pt.put("Battery", odometry.battery);
  /*** Write down running distance ***/
  pt.put("Distance", odometry.distance);
  /*** Write down current ***/
  pt_child.clear();
  for(int i = 0; i < odometry.current.size(); i++){
    prop::ptree p;
    p.put("", odometry.current[i]);
    pt_child.push_back(make_pair("", p));
  }
  pt.add_child("Current", pt_child);
  /*** Write down encoder count ***/
  pt_child.clear();
  for(int i = 0; i < odometry.encoder.size(); i++){
    prop::ptree p;
    p.put("", odometry.encoder[i]);
    pt_child.push_back(make_pair("", p));
  }
  pt.add_child("Encoder", pt_child);
  /*** Write down motor rpm ***/
  pt_child.clear();
  for(int i = 0; i < odometry.rpm.size(); i++){
    prop::ptree p;
    p.put("", odometry.rpm[i]);
    pt_child.push_back(make_pair("", p));
  }
  pt.add_child("RPM", pt_child);
  /*** Write down motor pwm ***/
  pt_child.clear();
  for(int i = 0; i < odometry.pwm.size(); i++){
    prop::ptree p;
    p.put("", odometry.pwm[i]);
    pt_child.push_back(make_pair("", p));
  }
  pt.add_child("PWM", pt_child);
  /*** Write down internal temperature ***/
  pt.put("Temperature.Internal", odometry.temperature);
  pt.put("Temperature.CPU", odometry.cpu_temperature);
  /*** Output json file ***/
  try{
    write_json(path, pt);
  } catch(...){
    return false;
  }
  return true;
}

bool pr::writePose(const std::string &path, const Pose3D &pose)
{
  prop::ptree pt;
  pt.put("Pose.X", pose.x);
  pt.put("Pose.Y", pose.y);
  pt.put("Pose.Z", pose.z);
  pt.put("Pose.Roll", RAD_TO_DEG * pose.roll);
  pt.put("Pose.Pitch", RAD_TO_DEG * pose.pitch);
  pt.put("Pose.Yaw", RAD_TO_DEG * pose.yaw);
  try{
    write_json(path, pt);
  } catch(...){
    return false;
  }
  return true;
}

bool pr::writePointCloud(const std::string &path, const cv::Point3f *points, const std::vector<float> &intensity)
{
  ofstream ofs(path);
  if(ofs.fail())return false;
  int size = intensity.size();
  ofs << "# .PCD v0.7 - Point Cloud Data file format" << endl;
  ofs << "VERSION 0.7" << endl;
  ofs << "FIELDS x y z intensity" << endl;
  ofs << "SIZE 4 4 4 4" << endl;
  ofs << "TYPE F F F F" << endl;
  ofs << "COUNT 1 1 1 1" << endl;
  ofs << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
  ofs << "POINTS " << size << endl;
  ofs << "DATA ascii" << endl;
  for(int i = 0; i < size; i++){
    ofs << points[i].x << " " 
        << points[i].y << " " 
        << points[i].z << " " 
        << intensity[i] << endl; 
  }
  return true;
}

bool pr::writePointCloud(const std::string &path, const std::vector<LidarData> &points)
{
  ofstream ofs(path);
  if(ofs.fail())return false;
  int size = points.size();
  ofs << "# .PCD v0.7 - Point Cloud Data file format" << endl;
  ofs << "VERSION 0.7" << endl;
  ofs << "FIELDS x y z intensity" << endl;
  ofs << "SIZE 4 4 4 4" << endl;
  ofs << "TYPE F F F F" << endl;
  ofs << "COUNT 1 1 1 1" << endl;
  ofs << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
  ofs << "POINTS " << size << endl;
  ofs << "DATA ascii" << endl;
  for(int i = 0; i < size; i++){
    ofs << points[i].x << " " 
        << points[i].y << " " 
        << points[i].z << " " 
        << points[i].reflectivity << endl; 
  }
  return true;
}

bool pr::writePointCloudBinary(const std::string &path, cv::Point3f *points, std::vector<float> &intensity)
{
  cout << "write binary in lib start" << endl;
  int size = intensity.size();
  FILE *fp = NULL;
  //fopen 関数はchar型のみに対応しているため　sring -> charへの変換をする
  fp = fopen(path.c_str(), "wb");
  //-----エラー処理-----
  if(fp == NULL){
    cerr << "Failed to open points file " << path << endl;
    return false;
  }
  //--------------------
  //-----文字列部分の読み飛ばし-----
  fprintf(fp, "%s\n", "# .PCD v0.7 - Point Cloud Data file format" );
  fprintf(fp, "%s\n", "VERSION 0.7");
  fprintf(fp, "%s\n", "FIELDS x y z intensity");
  fprintf(fp, "%s\n", "SIZE 4 4 4 4");
  fprintf(fp, "%s\n", "TYPE F F F F");
  fprintf(fp, "%s\n", "COUNT 1 1 1 1");
  fprintf(fp, "%s\n", "VIEWPOINT 0 0 0 1 0 0 0");
  fprintf(fp, "%s%d\n", "POINTS ", size);
  fprintf(fp, "%s\n", "DATA binary");
  //--------------------------------
  cout << "write binary in lib finish fprint" << endl;
  for(int i = 0; i < size; i++){
    float data[4];
    data[0] = points[i].x;
    data[1] = points[i].y;
    data[2] = points[i].z;
    data[3] = intensity[i];
    fwrite(&data, sizeof(float), 4, fp);
  }
  fclose(fp);
  return true;
  cout << "write binary in lib start" << endl;
}

bool pr::writePointCloudBinary(const std::string &path, std::vector<LidarData> &points)
{
  int size = points.size();
  FILE *fp;
  //fopen 関数はchar型のみに対応しているため　sring -> charへの変換をする
  fp = fopen(path.c_str(), "wb");
  //-----エラー処理-----
  if(fp == NULL){
    cerr << "Failed to open points file " << path << endl;
    return false;
  }
  //--------------------
  //-----文字列部分の読み飛ばし-----
  fprintf(fp, "%s\n", "# .PCD v0.7 - Point Cloud Data file format" );
  fprintf(fp, "%s\n", "VERSION 0.7");
  fprintf(fp, "%s\n", "FIELDS x y z intensity");
  fprintf(fp, "%s\n", "SIZE 4 4 4 4");
  fprintf(fp, "%s\n", "TYPE F F F F");
  fprintf(fp, "%s\n", "COUNT 1 1 1 1");
  fprintf(fp, "%s\n", "VIEWPOINT 0 0 0 1 0 0 0");
  fprintf(fp, "%s%d\n", "POINTS ", size);
  fprintf(fp, "%s\n", "DATA binary");
  //--------------------------------
  for(int i = 0; i < size; i++){
    float data[4];
    data[0] = points[i].x;
    data[1] = points[i].y;
    data[2] = points[i].z;
    data[3] = points[i].reflectivity;
    fwrite(&data, sizeof(float), 4, fp);
  }
  return true;
}

