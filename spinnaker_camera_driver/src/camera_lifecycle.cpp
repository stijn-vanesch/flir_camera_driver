#include <spinnaker_camera_driver/camera_lifecycle.hpp>


CameraLifecycle::CameraLifecycle(const std::string & node_name, const rclcpp::NodeOptions & options)
      : rclcpp_lifecycle::LifecycleNode(node_name, options)
              
{
  camera_ = std::make_shared<spinnaker_camera_driver::Camera>(this, "");

  if (!camera_->start()) {
    std::cout << "startup failed!" << std::endl;
  }
}

LifecycleCallbackReturn CameraLifecycle::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_configure() has been called.");

  if (!camera_->configure()) {
    RCLCPP_ERROR(get_logger(), "camera configuration failed.");
    return LifecycleCallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "camera parameters read.");

  camera_->startWrapper();

  // TODO start publishers, subscribers


  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn CameraLifecycle::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_activate() has been called.");

    if (!camera_->connectToCamera()) {
      RCLCPP_ERROR(get_logger(), "camera connection failed.");
      return LifecycleCallbackReturn::FAILURE;
    }
    

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
