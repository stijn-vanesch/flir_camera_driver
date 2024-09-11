#ifndef SPINNAKER_CAMERA_DRIVER__CAMERA_LIFECYCLE_HPP_
#define SPINNAKER_CAMERA_DRIVER__CAMERA_LIFECYCLE_HPP_

#include <memory>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <spinnaker_camera_driver/camera.hpp>

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace spinnaker_camera_driver
{

class CameraLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit CameraLifecycle(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<spinnaker_camera_driver::Camera> camera_;

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  
  LifecycleCallbackReturn on_activate (const rclcpp_lifecycle::State &state);

  LifecycleCallbackReturn on_deactivate (const rclcpp_lifecycle::State &state);

  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &state);
};

}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_LIFECYCLE_HPP_