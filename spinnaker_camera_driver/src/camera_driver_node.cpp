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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <spinnaker_camera_driver/camera_driver.hpp>
#include <spinnaker_camera_driver/camera_lifecycle.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // auto node = std::make_shared<spinnaker_camera_driver::CameraDriver>(rclcpp::NodeOptions());
  
  std::shared_ptr<spinnaker_camera_driver::CameraLifecycle> cameraLifecycle = std::make_shared<spinnaker_camera_driver::CameraLifecycle>("camera_driver_node", rclcpp::NodeOptions());
  
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(cameraLifecycle->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
