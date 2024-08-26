
#include <memory>
#include <image_transport/image_transport.hpp>
#include <spinnaker_camera_driver/camera.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CameraLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit CameraLifecycle(const std::string &node_name);
    ~CameraLifecycle();

private:
  std::shared_ptr<spinnaker_camera_driver::Camera> camera_;

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  
  LifecycleCallbackReturn on_activate (const rclcpp_lifecycle::State &state);

  LifecycleCallbackReturn on_deactivate (const rclcpp_lifecycle::State &state);

  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &state);
};