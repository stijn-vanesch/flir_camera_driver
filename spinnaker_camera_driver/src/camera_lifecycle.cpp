// Source file of the camera lifecycle node

#include <image_transport/image_transport.hpp>
#include <spinnaker_camera_driver/camera_lifecycle.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"


CameraLifecycle::CameraLifecycle(const std::string &node_name)
      : rclcpp_lifecycle::LifecycleNode(node_name,
          rclcpp::NodeOptions()
              .use_intra_process_comms(false)
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true))
              
{
    RCLCPP_INFO(get_logger(), "Node '%s' has been created.", get_name());
    camera_ = std::make_shared<spinnaker_camera_driver::Camera>(this, "");
}
CameraLifecycle::~CameraLifecycle() {}


LifecycleCallbackReturn CameraLifecycle::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_configure() has been called.");

    return LifecycleCallbackReturn::SUCCESS;
  }

LifecycleCallbackReturn CameraLifecycle::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_activate() has been called.");

    return LifecycleCallbackReturn::SUCCESS;
  } 

LifecycleCallbackReturn CameraLifecycle::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_deactivate() has been called.");

    return LifecycleCallbackReturn::SUCCESS;
  } 


LifecycleCallbackReturn CameraLifecycle::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_shutdown() has been called.");

    return LifecycleCallbackReturn::SUCCESS;
  } 


LifecycleCallbackReturn CameraLifecycle::on_error(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_shutdown() has been called.");

    return LifecycleCallbackReturn::SUCCESS;
  } 

// Initialize Camera Class
// std::shared_ptr<spinnaker_camera_driver::Camera> camera_;

// int main(int argc, char * argv[])
int main()
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
//   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // rclcpp::init(argc, argv);

  // rclcpp::executors::SingleThreadedExecutor exe;

//   std::shared_ptr<LifecycleTalker> lc_node =
//     std::make_shared<LifecycleTalker>("lc_talker");

//   exe.add_node(lc_node->get_node_base_interface());

//   exe.spin();

//   rclcpp::shutdown();

    std::cout << "Test Message" << std::endl;

  return 0;
}
