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

#ifndef SPINNAKER_CAMERA_DRIVER__CAMERA_LIFECYCLE_HPP_
#define SPINNAKER_CAMERA_DRIVER__CAMERA_LIFECYCLE_HPP_

#include <memory>
#include <spinnaker_camera_driver/camera.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

using LifecycleCallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace spinnaker_camera_driver
{

class CameraLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CameraLifecycle(const std::string & node_name, const rclcpp::NodeOptions & options);
  ~CameraLifecycle();

private:
  std::shared_ptr<spinnaker_camera_driver::Camera> camera_;

  // spinnaker_camera_driver::Camera* camera_;

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State & state);

  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State & state);

  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);

  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);

  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State & state);
};

}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_LIFECYCLE_HPP_
