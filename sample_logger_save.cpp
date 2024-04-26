//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

//-----------------------------
// Namespace & Using
//-----------------------------
using std_srvs::srv::Trigger;
using std_srvs::srv::SetBool;

//-----------------------------
// Class
//-----------------------------
namespace project_ryusei{
class ROS2LoggerSaveSignal: public rclcpp::Node{
public:
    ROS2LoggerSaveSignal(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~ROS2LoggerSaveSignal();
private:
    void run();
    std::unique_ptr<std::thread> m_thread;
};

//-----------------------------
// Methods
//-----------------------------
ROS2LoggerSaveSignal::ROS2LoggerSaveSignal(rclcpp::NodeOptions options): rclcpp::Node("rs_logger_save_signal", options)
{
    m_thread = std::make_unique<std::thread>(&ROS2LoggerSaveSignal::run, this);
    m_thread->detach();
}

ROS2LoggerSaveSignal::~ROS2LoggerSaveSignal()
{
    m_thread.release();
}

void ROS2LoggerSaveSignal::run()
{
    // loggerを起動
    RCLCPP_INFO(this->get_logger(), "send logger save signal.");
    auto logger_set_active_client = this->create_client<SetBool>("/rs_logger/set_active");
    auto logger_set_active_request = std::make_shared<SetBool::Request>();
    logger_set_active_request->data = true;
    auto logger_set_active_response = logger_set_active_client->async_send_request(logger_set_active_request);
    logger_set_active_response.wait_for(std::chrono::seconds(1));
    if(!logger_set_active_response.get()->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop logger");
        return;
    }

    // loggerのマニュアルセーブをリクエスト
    auto logger_trigger_save_client = this->create_client<Trigger>("/rs_logger/trigger_save");
    auto logger_trigger_save_request = std::make_shared<Trigger::Request>();
    auto logger_trigger_save_response = logger_trigger_save_client->async_send_request(logger_trigger_save_request);
    logger_trigger_save_response.wait();
    RCLCPP_INFO(this->get_logger(), "Complete! send logger save signal.");

    rclcpp::shutdown();
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(project_ryusei::ROS2LoggerSaveSignal)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------