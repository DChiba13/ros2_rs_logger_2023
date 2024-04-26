# ProjectRyusei.logger
This repository is logger program for Project Ryusei  

# src/logger.cpp & include/logger.hpp
loggerのメインプログラム．  
logファイルはパラメータ（log_root_path）で指定されたディレクトリ内に「sensor_YYYYmmdd_HHMMSS」フォルダを作成し，そのフォルダ内の各センサ用フォルダ内に保存される． 
なお，Manual操作によるログ保存については，各センサフォルダ内にあるManualフォルダに保存される

## Parameter
- log_root_path  
logファイルのメインディレクトリパス
- img_dir_name  
フロントカメラの画像ログ保存ディレクトリ名
- robot_status_dir_name  
Mercuryの状態ログ保存ディレクトリ名
- high_points_cloud_dir_name  
3D LiDAR（上）の点群データログ保存ディレクトリ名
- mid_points_cloud_dir_name  
3D LiDAR（中）の点群データログ保存ディレクトリ名
- low_points_cloud_dir_name  
2D LiDAR（下）の点群データログ保存ディレクトリ名
- location_dir_name  
LocatorのPoseログ保存ディレクトリ名
- sonar_dir_name  
ソナーセンサの距離ログ保存ディレクトリ名
- img_flag  
フロントカメラの画像ログ保存フラグ（true: 保存, false: 保存しない）
- robot_state_flag  
Mecuryの状態ログ保存フラグ（true: 保存, false: 保存しない）
- high_points_cloud_flag  
3D LiDAR（上）の点群データログ保存フラグ（true: 保存, false: 保存しない）
- mid_points_cloud_flag  
3D LiDAR（中）の点群データログ保存フラグ（true: 保存, false: 保存しない）
- low_points_cloud_flag  
2D LiDAR（下）の点群データログ保存フラグ（true: 保存, false: 保存しない）
- location_flag  
LocatorのPoseログ保存フラグ（true: 保存, false: 保存しない）
- sonar_flag  
ソナーセンサの距離ログ保存フラグ（true: 保存, false: 保存しない）  
- trigger_distance  
オートモード時のログ保存のトリガーとなる移動距離[m]
- logger_booting_mode  
ロガーの起動モード（0: 停止, 1: マニュアルモード, 2: オートモード）

## Subscriber
- /camera1/image (sensor_msgs::msg::Image)  
フロントカメラからの画像取得  
- /mercury/state (ros_mercury_interfaces::msg::MercuryStateMsg)  
Mercuryの状態取得  
- /c32/points (sensor_msgs::msg::PointCloud)  
3D LiDAR（上）の点群情報取得  
- /mrs1000/points (sensor_msgs::msg::PointCloud)  
3D LiDAR（中）の点群情報取得  
- /tim551/points (sensor_msgs::msg::PointCloud)  
2D LiDAR（下）の点群情報取得  
- /locator/corrected_pose (geometry_msgs::msg::Pose)  
Locatorからの位置情報取得
- /mercury_sonar/ranges  (std_msgs::msg::Float32MultiArray)  
ソナーセンサの距離取得

## Publisher
- /rs_logger/is_avtive  (std_msgs::msg::Int8)  
ロガー状態を配信（0: 停止, 1: マニュアルモード, 2: オートモード）

## Service(server)
- /rs_logger/set_active (std_srvs::srv::SetBoot)  
ロガーのオートモードON/OFF切替（true: ON，false: OFF）  
- /rs_logger/save_event  (std_srvs::srv::Empty)  
マニュアルモード時にログを保存

## Functions
- ROS2Logger::ROS2Logger()  
loggerのコンストラクタ  
・パラメータの初期化  
・ディレクトリの生成  
・サブスクライバの生成  
・パブリッシャの生成  
・サービスの生成  
・他スレッドでROSLogger::Runを実行

- ROS2Logger::~ROS2Logger()  
loggerのデストラクタ  
・threadのrelease 

- void ROS2Logger::on_____Subscribed()  
サブスクライバのコールバック関数  
・センサからの取得値を格納

- void ROS2Logger::service_____()  
サービスのコールバック関数  

- void ROS2Logger::Run()  
メインループ処理  
オートモードであれば，mercury/statusから得られたロボットの移動情報（distance）がtrigger_distance（パラメータ）以上変化したときに各センサのログを保存する  
・loop周期設定  
・ログファイル生成  
[以下，ループ処理]  
オートモード時において，mercury/statusからサブスクライブしたdistanceが前回のログ保存時からtrigger_distanceだけ移動していたらログを保存  
[以下，ループ後処理]  
・ファイルクローズ処理

# include/logger_cvt_functions.hpp
座標変換関数定義ファイル  

## Functions
- inline void QuaternionToEulerAngle()  
Quaternion角からEuler角への変換インライン関数    

# include/logger_file_function.hpp
ディレクトリ生成関数定義ファイル

## Functions
- inline bool CreateDirectory()  
指定したパスを生成するインライン関数  

# include/logger_save_functions.hpp
各メッセージ型のログ保存関数定義ファイル

## Functions
- inline void SaveMercuryState()  
MercuryStateMsg型のログ保存インライン関数  

- inline void SavePointCloud()  
PointCloud型のログ保存インライン関数  

- inline void SavePose()  
Pose型のログ保存インライン関数  

- inline void SaveFloat32MultiArray()
Float32Array型のログ保存インライン関数

# include/logger_def.hpp
loggerの状態定義ファイル

## constexpr & Enum
- LOGGER_SAVE
- LOGGER_STATUS

# src/sample_logger_save.cpp
loggerノードでマニュアルセーブするためのサンプルプログラム