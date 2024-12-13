// -*-c++-*--------------------------------------------------------------------
// Copyright 2024 Stijn van Esch <stijn.van.esch@wefabricate.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp_components/register_node_macro.hpp>
#include <spinnaker_camera_driver/camera_lifecycle.hpp>

namespace spinnaker_camera_driver
{

CameraLifecycle::CameraLifecycle(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("camera_driver_node", options)

{
  camera_ = std::make_shared<spinnaker_camera_driver::Camera>(
    static_cast<rclcpp_lifecycle::LifecycleNode *>(this), "");
}

CameraLifecycle::~CameraLifecycle() {}

LifecycleCallbackReturn CameraLifecycle::on_configure(const rclcpp_lifecycle::State &)
{
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
  RCLCPP_INFO(get_logger(), "on_error() has been called.");

  if (!camera_->destroyComponents()) {
    RCLCPP_ERROR(get_logger(), "camera destroy failed.");
    return LifecycleCallbackReturn::FAILURE;
  }

  return LifecycleCallbackReturn::SUCCESS;
}

}  // namespace spinnaker_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(spinnaker_camera_driver::CameraLifecycle)
