// dig_at_waypoint.hpp
#include "nav2_core/waypoint_task_executor.hpp"

class DumpAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  void onConfigure() override {}          // load params
  void onCleanup()  override {}
  void onActivate() override {}
  void onDeactivate() override {}

  bool execute(const geometry_msgs::msg::PoseStamped & pose,
               const uint32_t     waypoint_index) override
  {
    if (waypoint_index != 0) {            // we only dig at wp_0
      return true;                        // immediately OK
    }

    RCLCPP_INFO(logger_, "Digging at wp_%u", waypoint_index);
    // 1) call your digging action/service or publish to a dig controller
    dig_for_seconds_(dig_duration_sec_);
    // 2) return true when finished so Nav2 proceeds
    return true;
  }

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("DigAtWaypoint");
  double dig_duration_sec_{8.0};
};
