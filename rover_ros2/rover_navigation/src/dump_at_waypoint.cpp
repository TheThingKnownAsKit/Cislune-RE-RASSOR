// dig_at_waypoint.hpp
#include "nav2_core/waypoint_task_executor.hpp"

class DumpAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  void onConfigure() override {}          // load params
  void onCleanup()  override {}
  void onActivate() override {}
  void onDeactivate() override {}

  bool execute( const geometry_msgs::msg::PoseStamped &,
                             const uint32_t wp_idx )
{
  if (wp_idx != 0) return true;         // dig only at wp_0

  auto client = node_->create_client<std_srvs::srv::Trigger>(
                 "/digger_controller/activate");
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  rclcpp::sleep_for(8s);                // dig for 8 s
  client = node_->create_client<std_srvs::srv::Trigger>(
                 "/digger_controller/deactivate");
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  return true;                          // let Nav2 proceed to wp_1
}


private:
  rclcpp::Logger logger_ = rclcpp::get_logger("DigAtWaypoint");
  double dig_duration_sec_{8.0};
};
