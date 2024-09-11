#include <spinnaker_camera_driver/camera_lifecycle.hpp>


namespace spinnaker_camera_driver
{

CameraLifecycle::CameraLifecycle(const std::string & node_name, const rclcpp::NodeOptions & options)
      : rclcpp_lifecycle::LifecycleNode(node_name, options)

{}

LifecycleCallbackReturn CameraLifecycle::on_configure(const rclcpp_lifecycle::State &)
{

  camera_ = std::make_shared<spinnaker_camera_driver::Camera>(shared_from_this(), "");

  RCLCPP_INFO(get_logger(), "on_configure() has been called.");

  if (!camera_->configure()) {
    RCLCPP_ERROR(get_logger(), "camera configuration failed.");
    return LifecycleCallbackReturn::FAILURE;
  }

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn CameraLifecycle::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_activate() has been called.");

    if (!camera_->startAcquisition()) {
      RCLCPP_ERROR(get_logger(), "camera start failed.");
      return LifecycleCallbackReturn::FAILURE;
    }
    
    return LifecycleCallbackReturn::SUCCESS;
  } 

LifecycleCallbackReturn CameraLifecycle::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_deactivate() has been called.");

    if (!camera_->stopAcquisition()) {
      RCLCPP_ERROR(get_logger(), "camera stop failed.");
      return LifecycleCallbackReturn::FAILURE;
    }

    return LifecycleCallbackReturn::SUCCESS;
  } 


LifecycleCallbackReturn CameraLifecycle::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_shutdown() has been called.");

    if (!camera_->destroyComponents()) {
      RCLCPP_ERROR(get_logger(), "camera destroy failed.");
      return LifecycleCallbackReturn::FAILURE;
    }

    return LifecycleCallbackReturn::SUCCESS;
  } 

LifecycleCallbackReturn CameraLifecycle::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_cleanup() has been called.");
    
    if (!camera_->destroyComponents()) {
      RCLCPP_ERROR(get_logger(), "camera destroy failed.");
      return LifecycleCallbackReturn::FAILURE;
    }

    return LifecycleCallbackReturn::SUCCESS;
  }


LifecycleCallbackReturn CameraLifecycle::on_error(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_shutdown() has been called.");
    
    if (!camera_->destroyComponents()) {
      RCLCPP_ERROR(get_logger(), "camera destroy failed.");
      return LifecycleCallbackReturn::FAILURE;
    }

    return LifecycleCallbackReturn::SUCCESS;
  } 

}  // namespace spinnaker_camera_driver