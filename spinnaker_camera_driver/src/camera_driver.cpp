// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <spinnaker_camera_driver/camera_driver.hpp>
#include <spinnaker_camera_driver/logging.hpp>

namespace spinnaker_camera_driver
{
CameraDriver::CameraDriver(const rclcpp::NodeOptions & options) : Node("camera_driver", options)
{
  camera_ = std::make_shared<Camera>(static_cast<rclcpp::Node *>(this), "", true);
  // Startup camera
  if (!camera_->start()) {
    RCLCPP_INFO(get_logger(), "could not start camera!");
  }
}

CameraDriver::~CameraDriver() {}
}  // namespace spinnaker_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(spinnaker_camera_driver::CameraDriver)
