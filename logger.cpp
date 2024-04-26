//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <string>
#include <mutex>
#include <cstdio>
#include <ctime>
#include <memory>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>

#include "ros2_rs_miyauchi_logger/logger.hpp"
#include "ros2_rs_miyauchi_logger/logger_def.hpp"
#include "logger_lib.hpp"
// #include "defs.hpp"


#include "cvt_functions/quaternion_to_euler.hpp"
#include "filesystem/create_directory.hpp"
#include "filesystem/log_save_functions_v2.hpp"
#include "filesystem/get_home_dir.hpp"

#define VER_MIYA 0
#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)
//-----------------------------
// Namespace
//-----------------------------
namespace rs = project_ryusei;

//-----------------------------
// Methods
//-----------------------------
/**
 * @brief Project Ryusei
 * 
 */
namespace project_ryusei{

/**
 * @brief Construct a new ROS2Logger::ROS2Logger object
 * 
 * @param options 
 */
ROS2Logger::ROS2Logger(rclcpp::NodeOptions options): rclcpp::Node("rs_logger", options)
{
    // using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // パラメータ初期化
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_file_name                 = this->declare_parameter("logger.save_param.save_path", "rs_log/");
    m_log_save_root_path        = rs::get_homedir() + m_file_name;
    m_camera_img_save_dir       = this->declare_parameter("logger.save_param.img.dir", "image/");
    m_robot_state_save_dir      = this->declare_parameter("logger.save_param.robot_state.dir", "mercury_blue/");
    m_high_point_cloud_save_dir = this->declare_parameter("logger.save_param.high_point_cloud.dir", "pandar_40/");
    m_mid_point_cloud_save_dir  = this->declare_parameter("logger.save_param.mid_point_cloud.dir", "mid_pandar_40/");
    m_low_point_cloud_save_dir  = this->declare_parameter("logger.save_param.low_point_cloud.dir", "low_pandar_40/");
    m_location_save_dir         = this->declare_parameter("logger.save_param.location.dir", "location/");
    // m_sonar_range_save_dir      = this->declare_parameter("logger.save_param.sonar.dir", "mercury_sonar/");
    m_trigger_distance          = this->declare_parameter("logger.trigger.distance", 0.5);
    m_trigger_yaw               = this->declare_parameter("logger.trigger.yaw", 10.0) * M_PI / 180.0;
    m_logger_freq               = this->declare_parameter("logger.freq", 10);
    m_is_active                 = this->declare_parameter("logger.is_active", false);
    m_save_flag                |= this->declare_parameter("logger.save_param.img.flag", true)              * LOGGER_SAVE::IMG_FLAG;
    m_save_flag                |= this->declare_parameter("logger.save_param.robot_state.flag", true)      * LOGGER_SAVE::STATE_FLAG;
    m_save_flag                |= this->declare_parameter("logger.save_param.high_point_cloud.flag", true) * LOGGER_SAVE::HIGH_POINTS_FLAG;
    m_save_flag                |= this->declare_parameter("logger.save_param.mid_point_cloud.flag", true)  * LOGGER_SAVE::MID_POINTS_FLAG;
    m_save_flag                |= this->declare_parameter("logger.save_param.low_point_cloud.flag", true)  * LOGGER_SAVE::LOW_POINTS_FLAG;
    m_save_flag                |= this->declare_parameter("logger.save_param.location.flag", true)         * LOGGER_SAVE::LOCATION_FLAG;
    // m_save_flag                |= this->declare_parameter("logger.save_param.sonar.flag", true)            * LOGGER_SAVE::SONAR_FLAG;
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");
    RCLCPP_INFO(this->get_logger(), "LOGGER_SAVE::HIGH_POINTS_FLAG: %d", LOGGER_SAVE::HIGH_POINTS_FLAG);
    // ログ保存ディレクトリ生成用タイムスタンプ生成
    char date[64];
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);     // クロック取得
    std::time_t time = clock->now().seconds();                                          // タイムスタンプ取得
    std::strftime(date, sizeof(date),"%Y%m%d_%H%M%S", std::localtime(&time));           // フォーマット
    
    // ログファイル保存用フォルダ生成
    RCLCPP_INFO(this->get_logger(), "Create log save root directory...");
    m_log_save_root_path = m_log_save_root_path + "sensor_" + date + "/";               // ログファイル保存ディレクトリを時刻付きに変更
    if(!createDirectory(m_log_save_root_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to make directory: %s", m_log_save_root_path.c_str());
        RCLCPP_ERROR(this->get_logger(), "Shutdown logger.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Complete! Log save root directory was created.");

    // カメラからの取得画像保存フォルダ生成
    if(m_save_flag & LOGGER_SAVE::IMG_FLAG){
        m_camera_img_save_dir = m_log_save_root_path + m_camera_img_save_dir;
        if(!createDirectory(m_camera_img_save_dir) || !createDirectory(m_camera_img_save_dir + "Manual/") ) {
            RCLCPP_ERROR(this->get_logger(), "Failed to make directory[cameraImg]");
            RCLCPP_ERROR(this->get_logger(), "Shutdown logger.");
            return;
        }
    }
    
    // mercury状態ログ保存フォルダ生成
    if(m_save_flag & LOGGER_SAVE::STATE_FLAG){
        m_robot_state_save_dir = m_log_save_root_path + m_robot_state_save_dir;
        if(!createDirectory(m_robot_state_save_dir) || !createDirectory(m_robot_state_save_dir + "Manual/")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to make directory[robot state]");
            RCLCPP_ERROR(this->get_logger(), "Shutdown logger.");
            return;
        }
    }

    // 3D LiDARログ保存フォルダ生成
    if(m_save_flag & LOGGER_SAVE::HIGH_POINTS_FLAG){
        m_high_point_cloud_save_dir = m_log_save_root_path + m_high_point_cloud_save_dir;
        if(!createDirectory(m_high_point_cloud_save_dir) || !createDirectory(m_high_point_cloud_save_dir + "Manual/")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to make directory[high_point_cloud]");
            RCLCPP_ERROR(this->get_logger(), "Shutdown logger.");
            return;
        }
    }

    // 3D LiDAR（中）ログ保存フォルダ生成
    if(m_save_flag & LOGGER_SAVE::MID_POINTS_FLAG){
        m_mid_point_cloud_save_dir = m_log_save_root_path + m_mid_point_cloud_save_dir;
        if(!createDirectory(m_mid_point_cloud_save_dir) || !createDirectory(m_mid_point_cloud_save_dir + "Manual/")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to make directory[mid_point_cloud]");
            RCLCPP_ERROR(this->get_logger(), "Shutdown logger.");
            return;
        }
    }

    // 2D LiDAR（下）ログ保存フォルダ生成
    if(m_save_flag & LOGGER_SAVE::LOW_POINTS_FLAG){
        m_low_point_cloud_save_dir = m_log_save_root_path + m_low_point_cloud_save_dir;
        if(!createDirectory(m_low_point_cloud_save_dir) || !createDirectory(m_low_point_cloud_save_dir + "Manual/")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to make directory[low_point_cloud]");
            RCLCPP_ERROR(this->get_logger(), "Shutdown logger.");
            return;
        }
    }

    // ソナーセンサログ保存フォルダ
    // if(m_save_flag & LOGGER_SAVE::SONAR_FLAG){
    //     m_sonar_range_save_dir = m_log_save_root_path + m_sonar_range_save_dir;
    //     if(!createDirectory(m_sonar_range_save_dir) || !createDirectory(m_sonar_range_save_dir + "Manual/")) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to make directory[sonar]");
    //         RCLCPP_ERROR(this->get_logger(), "Shutdown logger.");
    //         return;
    //     }
    // }

    // locatorログ保存フォルダ生成
    if(m_save_flag & LOGGER_SAVE::LOCATION_FLAG){
        m_location_save_dir = m_log_save_root_path + m_location_save_dir;
        if(!createDirectory(m_location_save_dir) || !createDirectory(m_location_save_dir + "Manual/")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to make directory[location]");
            RCLCPP_ERROR(this->get_logger(), "Shutdown logger.");
            return;
        }
    }

    // サブスクライバ生成
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_robot_state_subscriber = this->create_subscription<RobotStateMsg>("/mercury/state", 10, std::bind(&ROS2Logger::onRobotStateSubScribed, this, _1));
    if(m_save_flag & LOGGER_SAVE::IMG_FLAG)         m_camera_img_subscriber       = this->create_subscription<Image>("/camera/image", 10, std::bind(&ROS2Logger::onCameraImgSubscribed, this, _1));
    if(m_save_flag & LOGGER_SAVE::HIGH_POINTS_FLAG) m_high_point_cloud_subscriber = this->create_subscription<PointCloud>("/top/points", 10, std::bind(&ROS2Logger::onHighPointCloudSubscribed, this, _1));
    if(m_save_flag & LOGGER_SAVE::MID_POINTS_FLAG)  m_mid_point_cloud_subscriber  = this->create_subscription<PointCloud>("/middle/points", 10, std::bind(&ROS2Logger::onMidPointCloudSubscribed, this, _1));
    if(m_save_flag & LOGGER_SAVE::LOW_POINTS_FLAG)  m_low_point_cloud_subscriber  = this->create_subscription<PointCloud>("/bottom/points", 10, std::bind(&ROS2Logger::onLowPointCloudSubscribed, this, _1));
    // if(m_save_flag & LOGGER_SAVE::SONAR_FLAG)       m_sonar_range_subscriber      = this->create_subscription<Float32MultiArray>("/sonar/ranges", 10, std::bind(&ROS2Logger::onSonarRangeSubscribed, this, _1));
    if(m_save_flag & LOGGER_SAVE::LOCATION_FLAG)    m_location_subscriber         = this->create_subscription<Pose>("/locator/corrected_pose", 10, std::bind(&ROS2Logger::onLocationSubScribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // パブリッシャ生成
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    m_is_active_publisher = this->create_publisher<Bool>("~/is_active", 10);
    auto_save_cnt_publisher_ = this->create_publisher<Int32>("~/auto_save_cnt", 10);
    manual_save_cnt_publisher_ = this->create_publisher<Int32>("~/manual_save_cnt", 10);
    file_name_publisher_ = this->create_publisher<String>("~/file_name",10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // サービス生成
    RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    m_set_active_server          = this->create_service<SetBool>("~/set_active", std::bind(&ROS2Logger::serviceSetActive, this, _1, _2, _3));
    m_trigger_manual_save_server = this->create_service<Trigger>("~/trigger_save", std::bind(&ROS2Logger::serviceTriggerManualSave, this, _1, _2, _3));
    RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // 処理開始
    m_thread = std::make_unique<std::thread>(&ROS2Logger::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the ROS2Logger::ROS2Logger object
 * 
 */
ROS2Logger::~ROS2Logger()
{
    m_thread.release();
}

void ROS2Logger::onCameraImgSubscribed(const Image::SharedPtr img)
{
    if(img->data.empty()) return;
    m_camera_img_mutex.lock();
    m_camera_img = img;
    m_camera_img_mutex.unlock();
}

void ROS2Logger::onRobotStateSubScribed(const RobotStateMsg::SharedPtr state)
{
    m_robot_state_mutex.lock();
    m_robot_state = state;
    m_robot_state_mutex.unlock();
}

void ROS2Logger::onLocationSubScribed(const Pose::SharedPtr pose)
{
    m_location_mutex.lock();
    m_location = pose;
    m_location_mutex.unlock();
}

void ROS2Logger::onHighPointCloudSubscribed(const PointCloud::SharedPtr point)
{
    if(point->points.empty()) return;
    m_high_point_cloud_mutex.lock();
    m_high_point_cloud = point;
    m_high_point_cloud_mutex.unlock();
}

void ROS2Logger::onMidPointCloudSubscribed(const PointCloud::SharedPtr point)
{
    if(point->points.empty()) return;
    m_mid_point_cloud_mutex.lock();
    m_mid_point_cloud = point;
    m_mid_point_cloud_mutex.unlock();
}

void ROS2Logger::onLowPointCloudSubscribed(const PointCloud::SharedPtr point)
{
    if(point->points.empty()) return;
    m_low_point_cloud_mutex.lock();
    m_low_point_cloud = point;
    m_low_point_cloud_mutex.unlock();
}

// void ROS2Logger::onSonarRangeSubscribed(const Float32MultiArray::SharedPtr range)
// {
//     if(range->data.empty()) return;
//     m_sonar_range_mutex.lock();
//     m_sonar_range = range;
//     m_sonar_range_mutex.unlock();
// }

void ROS2Logger::serviceSetActive(const std::shared_ptr<rmw_request_id_t> header,
                                    const SetBool::Request::SharedPtr req,
                                    const SetBool::Response::SharedPtr res)
{

    m_is_active_mutex.lock();
    m_is_active = req->data;
    m_is_active_mutex.unlock();
    res->success = true;
    res->message = "Set logger active state";
}

void ROS2Logger::serviceTriggerManualSave(const std::shared_ptr<rmw_request_id_t> header, 
                                            const Trigger::Request::SharedPtr req, 
                                            const Trigger::Response::SharedPtr res)
{
    m_manual_save_mutex.lock();
    m_manual_save = true;
    m_manual_save_mutex.unlock();
    res->success = true;
    res->message = "Manual save!";
}

/**
 * @brief execute method
 * 
 * loggerのメイン処理
 * 
 * PointCloud型およびImage型については，save回数をファイル名として保存する（パターンA）．
 * それ以外については，1つのファイルにすべてのログを保存する（パターンB）．
 * 
 * パターンAについては，
 * saveイベントが発生するたびにファイルを生成し，m_manual_saveがtrueのときには，各種センサのManualフォルダに保存する．
 * falseの場合には，各種センサのフォルダ内に保存する．
 * これらの切り替えは，配列（LOGGER_SAVE_DIR[]）を用いて行っており，false=0, true=1という性質を利用している．
 * 
 * パターンBについては，
 * あらかじめ，ログファイルを配列で生成し，0番目をAuto用のファイル，1番目をManual用のファイルとしている．
 * こちらについても，false=0, true=1という性質を利用して，Manualに保存するかを切り替えている．
 * 
 */
void ROS2Logger::run()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "%s has started. thread id = %0x", this->get_name(), std::this_thread::get_id());
    
    // sonarログファイル生成
    // std::ofstream sonar_range_log_file[2];
    // if(m_save_flag & LOGGER_SAVE::SONAR_FLAG){
    //     const std::string &header = "range1, range2, range3, range4, range5, range6, range7";
    //     sonar_range_log_file[0].open(m_sonar_range_save_dir + "/mercury_sonar.csv");
    //     sonar_range_log_file[0] << header << std::endl;

    //     sonar_range_log_file[1].open(m_sonar_range_save_dir + "/Manual/mercury_sonar.csv");
    //     sonar_range_log_file[1] << header << std::endl;
    // }
    // std::cout << "logger_debug" << std::endl;
    // RCLCPP_INFO(this->get_logger(), "logger_debug");
    

    // データ到達まで待機
    for(rclcpp::WallRate loop(10); this->getRobotState().get() == nullptr; loop.sleep());
    while((m_save_flag & LOGGER_SAVE::IMG_FLAG) && this->getCameraImg().get() == nullptr);
    while((m_save_flag & LOGGER_SAVE::MID_POINTS_FLAG) && this->getMidPointCloud().get() == nullptr);
    //sita2gyoudetomaru
    while((m_save_flag & LOGGER_SAVE::HIGH_POINTS_FLAG) && this->getHighPointCloud().get() == nullptr);
    while((m_save_flag & LOGGER_SAVE::LOW_POINTS_FLAG) && this->getLowPointCloud().get() == nullptr);
    while((m_save_flag & LOGGER_SAVE::LOCATION_FLAG) && this->getLocation().get() == nullptr);
    // while((m_save_flag & LOGGER_SAVE::SONAR_FLAG) && this->getSonarRange().get() == nullptr);

    // パブリッシュデータ
    Bool is_active_msg;
    Int32 auto_save_cnt_msg;
    Int32 manual_save_cnt_msg;
    String file_name_msg;
    file_name_msg.data = m_file_name;

    // ログ保存関連
    int auto_save_cnt   = 0;
    int manual_save_cnt = 0;
    double trigger_distance_next = m_trigger_distance;
    double robot_state_yaw_pre = 0.0;

        m_logger_freq = 20;
    for(rclcpp::WallRate loop(m_logger_freq); rclcpp::ok(); loop.sleep()){
        // アクティブ状態のラッチ

        m_is_active_mutex.lock();
        is_active_msg.data = m_is_active;
        // std::cout << "logger is_active: "  << std::endl;
        m_is_active_mutex.unlock();

        // 手動保存のラッチ
        m_manual_save_mutex.lock();
        bool manual_save = m_manual_save;
        m_manual_save = false;
        m_manual_save_mutex.unlock();

        // ロボット状態のラッチ
        auto robot_state = this->getRobotState();

        double roll, pitch, robot_state_yaw;
        rs::quaternionToEulerAngle(robot_state->odometry.pose.pose.orientation, &roll, &pitch, &robot_state_yaw);
        double diff_yaw = robot_state_yaw - robot_state_yaw_pre;
        if(M_PI < diff_yaw)       diff_yaw -= 2 * M_PI;             //  180[deg]超のときは，反対周り
        else if(diff_yaw < -M_PI) diff_yaw += 2 * M_PI;             // -180[deg]超のときは，反対周り

        // autoモード かつ 前回のlog保存からtrigger_distance以上進んでいる, trigger_yawだけ旋回している
        const bool &auto_save = is_active_msg.data
                                && (trigger_distance_next <= robot_state->distance || std::abs(diff_yaw) > m_trigger_yaw);

        // auto時のsaveイベント発生 もしくは manual時のsaveイベント発生
        if(auto_save || manual_save){
            char stamp[8];
            if(manual_save) std::snprintf(stamp, sizeof(stamp), "%06d", manual_save_cnt++);
            else{
                std::snprintf(stamp, sizeof(stamp), "%06d", auto_save_cnt++);
                trigger_distance_next = robot_state->distance + m_trigger_distance; // 次のトリガー
                robot_state_yaw_pre = robot_state_yaw;                              // 次の計算用
            }

            // カメラ取得画像保存
            if(m_save_flag & LOGGER_SAVE::IMG_FLAG){
                const auto &img = this->getCameraImg();
                cv::imwrite(m_camera_img_save_dir + LOGGER_SAVE_DIR[manual_save] + stamp + ".png", cv_bridge::toCvShare(img, img->encoding)->image);
            }
            
            // mercury state保存
            if(m_save_flag & LOGGER_SAVE::STATE_FLAG){
                const auto &state = this->getRobotState();
                std::string robot_state_str(m_robot_state_save_dir + LOGGER_SAVE_DIR[manual_save] + stamp + ".json");
                rs::Odometry odometry;
                double roll,pitch, yaw;
                double imu_roll,imu_pitch,imu_yaw;
                //とりあえず型合わせるために代入
                rs::quaternionToEulerAngle(state->odometry.pose.pose.orientation, &roll, &pitch, &yaw);
                rs::quaternionToEulerAngle(state->odometry.pose.pose.orientation, &imu_roll, &imu_pitch, &imu_yaw);
                odometry.pose.x = state->odometry.pose.pose.position.x;
                odometry.pose.y = state->odometry.pose.pose.position.y;
                odometry.pose.z = state->odometry.pose.pose.position.z;
                odometry.pose.roll = roll;
                odometry.pose.pitch = pitch;
                odometry.pose.yaw = yaw;
                odometry.velocity.x = state->odometry.twist.twist.linear.x;
                odometry.velocity.y = state->odometry.twist.twist.linear.y;
                odometry.velocity.z = state->odometry.twist.twist.linear.z;
                odometry.velocity.roll = state->odometry.twist.twist.angular.x;
                odometry.velocity.pitch = state->odometry.twist.twist.angular.y;
                odometry.velocity.yaw = state->odometry.twist.twist.angular.z;
                odometry.acceleration.x = state->imu.linear_acceleration.x;
                odometry.acceleration.y = state->imu.linear_acceleration.y;
                odometry.acceleration.z = state->imu.linear_acceleration.z;
                odometry.battery = state->battery;
                odometry.distance = state->distance;
                odometry.temperature = state->temperature;
                odometry.cpu_temperature = state->cpu_temperature;
                    for(int i = 0; i < 4; i++){
                        odometry.current.push_back(state->current[i]);
                        odometry.encoder.push_back(state->encoder[i]);
                        odometry.rpm.push_back(state->rpm[i]);
                    }
                rs::writeOdometry(robot_state_str,odometry);
            };


            // 3D LiDAR保存
            if(m_save_flag & LOGGER_SAVE::HIGH_POINTS_FLAG){
                std::string high_points_str(m_high_point_cloud_save_dir + LOGGER_SAVE_DIR[manual_save] + stamp + ".pcd");
                const auto &high_points = this->getHighPointCloud();
                int points_size = high_points->channels[1].values.size();
                cv::Point3f points[points_size];
                std::vector<float>intensity(points_size);
                for(int i = 0; i < points_size; i++){
                    points[i].x = high_points->points[i].x;
                    points[i].y = high_points->points[i].y;
                    points[i].z = high_points->points[i].z;
                    intensity[i] = high_points->channels[1].values[i];
                }
                rs::writePointCloudBinary(high_points_str, points, intensity);
                // rs::writePointCloud(high_points_str, points, intensity);
            }

            // 3D LiDAR（中）保存
            if(m_save_flag & LOGGER_SAVE::MID_POINTS_FLAG){
                std::string mid_points_str(m_mid_point_cloud_save_dir + LOGGER_SAVE_DIR[manual_save] + stamp + ".pcd");
                const auto &mid_points = this->getMidPointCloud();
                RCLCPP_INFO(this->get_logger(), "logger where in save");
                int points_size = mid_points->channels[1].values.size(); 
                cv::Point3f points[points_size];
                std::vector<float>intensity(points_size);
                for(int i = 0; i < points_size; i++){
                    points[i].x = mid_points->points[i].x;
                    points[i].y = mid_points->points[i].y;
                    points[i].z = mid_points->points[i].z;
                    intensity[i] = mid_points->channels[1].values[i];
                }

                RCLCPP_INFO(this->get_logger(), "write binary start");
                
                rs::writePointCloudBinary(mid_points_str, points, intensity);
                RCLCPP_INFO(this->get_logger(), "write binary finish");
                // rs::writePointCloud(mid_points_str, points, intensity);
            }

            // 2D LiDAR（下）保存
            if(m_save_flag & LOGGER_SAVE::LOW_POINTS_FLAG){
                std::string low_points_str(m_low_point_cloud_save_dir + LOGGER_SAVE_DIR[manual_save] + stamp + ".pcd");
                const auto &low_points = this->getLowPointCloud();
                int points_size = low_points->channels[1].values.size(); 
                cv::Point3f points[points_size];
                std::vector<float>intensity(points_size);
                for(int i = 0; i < points_size; i++){
                    points[i].x = low_points->points[i].x;
                    points[i].y = low_points->points[i].y;
                    points[i].z = low_points->points[i].z;
                    intensity[i] = low_points->channels[1].values[i];
                }
                rs::writePointCloudBinary(low_points_str, points, intensity);
                // rs::writePointCloud(low_points_str, points, intensity);
            }

            // Location保存
            if(m_save_flag & LOGGER_SAVE::LOCATION_FLAG){
                const auto &location = this->getLocation();
                //ここエラーです
                std::string location_str(m_location_save_dir + LOGGER_SAVE_DIR[manual_save] + stamp + ".json");
                rs::Pose3D pose;
                pose.x = location->position.x;
                pose.y = location->position.y;
                pose.z = location->position.z;
                // pose.roll = location->orientation.x;
                // pose.pitch = location->orientation.y;
                // pose.yaw = location->orientation.z;
                 

                rs::quaternionToEulerAngle(location->orientation, &pose.roll, &pose.pitch, &pose.yaw);
                rs::writePose(location_str, pose);
            }
            
            // sonar保存
            // if(m_save_flag & LOGGER_SAVE::SONAR_FLAG) rs::saveFloat32MultiArray(sonar_range_log_file[manual_save], this->getSonarRange());
        }
        // saveイベント未発生時にstateをpub
        else{
            auto_save_cnt_msg.data = auto_save_cnt;
            manual_save_cnt_msg.data = manual_save_cnt;
            auto_save_cnt_publisher_->publish(auto_save_cnt_msg);
            manual_save_cnt_publisher_->publish(manual_save_cnt_msg);
            m_is_active_publisher->publish(is_active_msg);
            file_name_msg.data = m_file_name;
            file_name_publisher_->publish(file_name_msg);
        }
    }

    // ファイルクローズ
    // sonar_range_log_file[0].close();
    // sonar_range_log_file[1].close();

    // ファイル数表示ファイル生成
    std::ofstream(m_log_save_root_path + "auto_" + std::to_string(auto_save_cnt));
    std::ofstream(m_log_save_root_path + "manual_" + std::to_string(manual_save_cnt));

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(project_ryusei::ROS2Logger)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------