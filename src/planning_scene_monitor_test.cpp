#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/macros/console_colors.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_test.planning_scene_monitor_test");

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("planning_scene_monitor_test");

  node->declare_parameter(ROBOT_DESCRIPTION, "");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "robot_state_publisher");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto urdf_xml = parameters_client->get_parameter<std::string>(ROBOT_DESCRIPTION, "");

  if (urdf_xml.empty())
    RCLCPP_ERROR(LOGGER, "Failed to read robot_description parameter");

  node->set_parameter(rclcpp::Parameter(ROBOT_DESCRIPTION, urdf_xml));

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
  std::shared_ptr<tf2_ros::TransformListener> tfl = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
      new planning_scene_monitor::PlanningSceneMonitor(node, ROBOT_DESCRIPTION, tf_buffer));

  if (planning_scene_monitor->getPlanningScene())
  {
    bool debug = false;
    for (int i = 1; i < argc; ++i)
      if (strncmp(argv[i], "--debug", 7) == 0)
      {
        debug = true;
        break;
      }
    if (debug)
      RCLCPP_INFO(LOGGER, "MoveGroup debug mode is ON");
    else
      RCLCPP_INFO(LOGGER, "MoveGroup debug mode is OFF");

    printf(MOVEIT_CONSOLE_COLOR_CYAN "Starting planning scene monitors...\n" MOVEIT_CONSOLE_COLOR_RESET);
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();
    printf(MOVEIT_CONSOLE_COLOR_CYAN "Planning scene monitors started.\n" MOVEIT_CONSOLE_COLOR_RESET);

    planning_scene_monitor->publishDebugInformation(debug);
  }
  else
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");

  rclcpp::WallRate loop_rate(30);
    while (rclcpp::ok()) {

        // rclcpp::spin_some(node);
        loop_rate.sleep();
    }

  return 0;
}