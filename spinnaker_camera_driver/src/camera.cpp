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

#include "spinnaker_camera_driver/camera.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <spinnaker_camera_driver/exposure_controller.hpp>
#include <spinnaker_camera_driver/logging.hpp>
#include <type_traits>

namespace spinnaker_camera_driver
{
namespace chrono = std::chrono;
//
// this complicated code is to detect an interface change
// between foxy and galactic
// See  https://stackoverflow.com/questions/1005476/
//  how-to-detect-whether-there-is-a-specific-member-variable-in-class

template <typename T, typename = bool>
struct DescSetter
{
  // don't set by default (foxy)
  static void set_dynamic_typing(T *) {}
};

template <typename T>
struct DescSetter<T, decltype((void)T::dynamic_typing, true)>
{
  // set if dynamic_typing is present
  static void set_dynamic_typing(T * desc) { desc->dynamic_typing = true; }
};

static rcl_interfaces::msg::ParameterDescriptor make_desc(const std::string name, int type)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = name;
  desc.type = type;
  desc.description = name;
  DescSetter<rcl_interfaces::msg::ParameterDescriptor>::set_dynamic_typing(&desc);
  return (desc);
}

static std::pair<bool, double> get_double_int_param(const rclcpp::Parameter & p)
{
  std::pair<bool, double> bd(false, 0);
  if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
    bd.second = p.as_double();
    bd.first = true;
  }
  if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
    bd.second = static_cast<double>(p.as_int());
    bd.first = true;
  }
  return (bd);
}

static std::pair<bool, bool> get_bool_int_param(const rclcpp::Parameter & p)
{
  std::pair<bool, bool> bb(false, false);
  if (p.get_type() == rclcpp::PARAMETER_BOOL) {
    bb.second = p.as_bool();
    bb.first = true;
  }
  if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
    bb.second = static_cast<bool>(p.as_int());
    bb.first = true;
  }
  return (bb);
}

Camera::NodeInfo::NodeInfo(const std::string & n, const std::string & nodeType) : name(n)
{
  if (nodeType == "float") {
    type = FLOAT;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_DOUBLE);
  } else if (nodeType == "int") {
    type = INT;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_INTEGER);
  } else if (nodeType == "bool") {
    type = BOOL;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_BOOL);
  } else if (nodeType == "enum") {
    type = ENUM;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_STRING);
  } else if (nodeType == "command") {
    type = COMMAND;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_NOT_SET);
  }
}

template <typename NodeT>
Camera::Camera(NodeT node, const std::string & prefix, bool useStatus)
{
  node_base = node->get_node_base_interface();
  node_timer = node->get_node_timers_interface();
  node_clock = node->get_node_clock_interface();
  node_logging = node->get_node_logging_interface();
  node_parameters = node->get_node_parameters_interface();
  node_topics = node->get_node_topics_interface();

  name_ = prefix;
  prefix_ = prefix.empty() ? std::string("") : (prefix + ".");
  topicPrefix_ = prefix.empty() ? std::string("") : (prefix + "/");
  lastStatusTime_ = node_clock->get_clock()->now();
  if (useStatus) {
    statusTimer_ = rclcpp::create_timer(
      node_base, node_timer, node_clock->get_clock(), rclcpp::Duration(5, 0),
      std::bind(&Camera::printStatus, this));
  }
}

Camera::~Camera()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

bool Camera::stop()
{
  stopCamera();
  if (wrapper_) {
    wrapper_->deInitCamera();
  }
  if (statusTimer_ && !statusTimer_->is_canceled()) {
    statusTimer_->cancel();
  }
  keepRunning_ = false;
  if (thread_) {
    thread_->join();
    thread_ = 0;
  }
  return (true);
}

bool Camera::stopCamera()
{
  if (cameraRunning_ && wrapper_) {
    cameraRunning_ = false;
    return wrapper_->stopCamera();
  }
  return false;
}

void Camera::printStatus()
{
  if (wrapper_) {
    const double dropRate =
      (queuedCount_ > 0) ? (static_cast<double>(droppedCount_) / static_cast<double>(queuedCount_))
                         : 0;
    const rclcpp::Time t = node_clock->get_clock()->now();
    const rclcpp::Duration dt = t - lastStatusTime_;
    const double dtns = std::max(dt.nanoseconds(), (int64_t)1);
    const double outRate = publishedCount_ * 1e9 / dtns;
    const double incompleteRate = wrapper_->getIncompleteRate();
    if (incompleteRate != 0) {
      LOG_WARN_FMT(
        "rate [Hz] in %6.2f out %6.2f drop %3.0f%% INCOMPLETE %3.0f%%",
        wrapper_->getReceiveFrameRate(), outRate, dropRate * 100, incompleteRate * 100);
    } else {
      LOG_INFO_FMT(
        "rate [Hz] in %6.2f out %6.2f drop %3.0f%%", wrapper_->getReceiveFrameRate(), outRate,
        dropRate * 100);
    }
    lastStatusTime_ = t;
    droppedCount_ = 0;
    publishedCount_ = 0;
    queuedCount_ = 0;
  } else {
    LOG_WARN("camera is not online!");
  }
}

void Camera::checkSubscriptions()
{
  if (connectWhileSubscribed_) {
    if (imagePub_->get_subscription_count() > 0 || metaPub_->get_subscription_count() > 0) {
      if (!cameraRunning_) {
        startCamera();
      }
    } else {
      if (cameraRunning_) {
        stopCamera();
      }
    }
  }
}

void Camera::readParameters()
{
  quiet_ = safe_declare<bool>(prefix_ + "quiet", false);
  serial_ = safe_declare<std::string>(prefix_ + "serial_number", "missing_serial_number");
  if (!quiet_) {
    LOG_INFO("reading ros parameters for camera with serial: " << serial_);
  }
  debug_ = safe_declare<bool>(prefix_ + "debug", false);
  adjustTimeStamp_ = safe_declare<bool>(prefix_ + "adjust_timestamp", false);
  if (!quiet_) {
    LOG_INFO((adjustTimeStamp_ ? "" : "not ") << "adjusting time stamps!");
  }

  cameraInfoURL_ =
    safe_declare<std::string>(prefix_ + "camerainfo_url", "../camera_params_23306238.yaml");
  frameId_ = safe_declare<std::string>(prefix_ + "frame_id", node_base->get_name());
  dumpNodeMap_ = safe_declare<bool>(prefix_ + "dump_node_map", false);
  qosDepth_ = safe_declare<int>(prefix_ + "image_queue_size", 4);
  maxBufferQueueSize_ = static_cast<size_t>(safe_declare<int>(prefix_ + "buffer_queue_size", 4));
  computeBrightness_ = safe_declare<bool>(prefix_ + "compute_brightness", false);
  acquisitionTimeout_ = safe_declare<double>(prefix_ + "acquisition_timeout", 3.0);
  parameterFile_ = safe_declare<std::string>(prefix_ + "parameter_file", "parameters.yaml");
  connectWhileSubscribed_ = safe_declare<bool>(prefix_ + "connect_while_subscribed", false);
  enableExternalControl_ = safe_declare<bool>(prefix_ + "enable_external_control", false);
  callbackHandle_ = node_parameters->add_on_set_parameters_callback(
    std::bind(&Camera::parameterChanged, this, std::placeholders::_1));
}

bool Camera::readParameterDefinitionFile()
{
  LOG_INFO("parameter definitions file: " << parameterFile_);
  YAML::Node yamlFile = YAML::LoadFile(parameterFile_);
  if (yamlFile.IsNull()) {
    LOG_ERROR("cannot open file: " << parameterFile_);
    return (false);
  }
  if (!yamlFile["parameters"].IsSequence()) {
    LOG_ERROR("parameter definitions lists no parameters!");
    return (false);
  }
  YAML::Node params = yamlFile["parameters"];
  for (const auto & p : params) {
    if (!p["name"]) {
      LOG_WARN("ignoring parameter missing name: " << p);
      continue;
    }
    if (!p["type"]) {
      LOG_WARN("ignoring parameter missing type: " << p);
      continue;
    }
    if (!p["node"]) {
      LOG_WARN("ignoring parameter missing node: " << p);
      continue;
    }
    const std::string pname = prefix_ + p["name"].as<std::string>();
    parameterMap_.insert(
      {pname, NodeInfo(p["node"].as<std::string>(), p["type"].as<std::string>())});
    parameterList_.push_back(pname);
  }
  return (true);
}

void Camera::createCameraParameters()
{
  for (const auto & name : parameterList_) {
    const auto it = parameterMap_.find(name);
    if (it != parameterMap_.end()) {
      const auto & ni = it->second;  // should always succeed
      safe_declare(name, rclcpp::ParameterValue(), ni.descriptor);
    }
  }
}

bool Camera::setEnum(const std::string & nodeName, const std::string & v)
{
  if (!quiet_) {
    LOG_INFO("setting " << nodeName << " to: " << v);
  }
  std::string retV;  // what actually was set
  std::string msg = wrapper_->setEnum(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    LOG_WARN("setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (v != retV) {
    LOG_WARN(nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

bool Camera::setDouble(const std::string & nodeName, double v)
{
  if (!quiet_) {
    LOG_INFO("setting " << nodeName << " to: " << v);
  }
  double retV;  // what actually was set
  std::string msg = wrapper_->setDouble(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    LOG_WARN("setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (std::abs(v - retV) > 0.025 * std::abs(v + retV)) {
    LOG_WARN(nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

bool Camera::setInt(const std::string & nodeName, int v)
{
  if (!quiet_) {
    LOG_INFO("setting " << nodeName << " to: " << v);
  }
  int retV;  // what actually was set
  std::string msg = wrapper_->setInt(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    LOG_WARN("setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (v != retV) {
    LOG_WARN(nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

bool Camera::setBool(const std::string & nodeName, bool v)
{
  if (!quiet_) {
    LOG_INFO("setting " << nodeName << " to: " << v);
  }
  bool retV;  // what actually was set
  std::string msg = wrapper_->setBool(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    LOG_WARN("setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (v != retV) {
    LOG_WARN(nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

bool Camera::execute(const std::string & nodeName)
{
  if (!quiet_) {
    LOG_INFO("executing " << nodeName);
  }
  std::string msg = wrapper_->execute(nodeName);
  if (msg != "OK") {
    LOG_WARN("executing " << nodeName << " failed: " << msg);
    return false;
  }

  return true;
}

void Camera::setParameter(const NodeInfo & ni, const rclcpp::Parameter & p)
{
  switch (ni.type) {
    case NodeInfo::ENUM: {
      std::string s = p.value_to_string();
      // remove quotes
      s.erase(remove(s.begin(), s.end(), '\"'), s.end());
      setEnum(ni.name, s);
      break;
    }
    case NodeInfo::FLOAT: {
      auto bd = get_double_int_param(p);
      if (bd.first) {
        setDouble(ni.name, bd.second);
      } else {
        LOG_WARN("bad non-float " << p.get_name() << " type: " << p.get_type());
      }
      break;
    }
    case NodeInfo::INT: {
      auto bd = get_double_int_param(p);
      if (bd.first) {
        setInt(ni.name, bd.second);
      } else {
        LOG_WARN("bad non-int " << p.get_name() << " type: " << p.get_type());
      }
      break;
    }
    case NodeInfo::BOOL: {
      auto bb = get_bool_int_param(p);
      if (bb.first) {
        setBool(ni.name, bb.second);
      } else {
        LOG_WARN("bad non-bool " << p.get_name() << " type: " << p.get_type());
      }
      break;
    }
    case NodeInfo::COMMAND: {
      execute(ni.name);
      break;
    }
    default:
      LOG_WARN("invalid node type in map: " << ni.type);
  }
}

rcl_interfaces::msg::SetParametersResult Camera::parameterChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    const auto it = parameterMap_.find(p.get_name());
    if (it == parameterMap_.end()) {
      continue;  // ignore unknown param
    }
    if (!wrapper_) {
      LOG_WARN("got parameter update while driver is not ready!");
      continue;
    }
    const NodeInfo & ni = it->second;
    if (p.get_type() == rclcpp::PARAMETER_NOT_SET) {
      continue;
    }
    try {
      setParameter(ni, p);
    } catch (const spinnaker_camera_driver::SpinnakerWrapper::Exception & e) {
      LOG_WARN("param " << p.get_name() << " " << e.what());
    }
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  res.reason = "all good!";
  return (res);
}

void Camera::controlCallback(const flir_camera_msgs::msg::CameraControl::UniquePtr msg)
{
  const uint32_t et = msg->exposure_time;
  const float gain = msg->gain;
  bool logTime(false);
  bool logGain(false);
  try {
    if (et > 0 && et != currentExposureTime_) {
      const auto it = parameterMap_.find(prefix_ + "exposure_time");
      if (it != parameterMap_.end()) {
        const auto & ni = it->second;
        setDouble(ni.name, et);
        currentExposureTime_ = et;
        logTime = true;
      } else {
        LOG_WARN("no node name defined for exposure_time, check .cfg file!");
      }
    }
    if (gain > std::numeric_limits<float>::lowest() && gain != currentGain_) {
      const auto it = parameterMap_.find(prefix_ + "gain");
      if (it != parameterMap_.end()) {
        const auto & ni = it->second;
        setDouble(ni.name, gain);
        currentGain_ = gain;
        logGain = true;
      } else {
        LOG_WARN("no node name defined for exposure_time, check .cfg file!");
      }
    }
  } catch (const spinnaker_camera_driver::SpinnakerWrapper::Exception & e) {
    LOG_WARN("failed to control: " << e.what());
  }

  if (logTime) {
    LOG_INFO("changed exposure time to " << et << "us");
  }
  if (logGain) {
    LOG_INFO("changed gain to " << gain << "db");
  }
}

void Camera::processImage(const ImageConstPtr & im)
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queuedCount_++;
    if (bufferQueue_.size() < maxBufferQueueSize_) {
      bufferQueue_.push_back(im);
      cv_.notify_all();
    } else {
      droppedCount_++;
    }
  }
}

void Camera::run()
{
  while (keepRunning_ && rclcpp::ok()) {
    {
      ImageConstPtr img;
      {  // ------- locked section ---
        std::unique_lock<std::mutex> lock(mutex_);
        // one second timeout
        const std::chrono::microseconds timeout((int64_t)(1000000LL));
        while (bufferQueue_.empty() && keepRunning_ && rclcpp::ok()) {
          cv_.wait_for(lock, timeout);
        }
        if (!bufferQueue_.empty()) {
          img = bufferQueue_.back();
          bufferQueue_.pop_back();
        }
      }  // -------- end of locked section
      if (img && keepRunning_ && rclcpp::ok()) {
        doPublish(img);
        if (exposureController_) {
          exposureController_->update(this, img);
        }
      }
    }
  }
}

using flir_fmt = spinnaker_camera_driver::pixel_format::PixelFormat;
namespace ros_fmt = sensor_msgs::image_encodings;

static const std::unordered_map<flir_fmt, std::string> flir_2_ros{
  {{flir_fmt::INVALID, "INV"},
   {flir_fmt::Mono8, ros_fmt::MONO8},
   {flir_fmt::Mono10p, "INV"},
   {flir_fmt::Mono10Packed, "INV"},
   {flir_fmt::Mono12p, "INV"},
   {flir_fmt::Mono12Packed, "INV"},
   {flir_fmt::Mono16, ros_fmt::MONO16},
   {flir_fmt::BayerRG8, ros_fmt::BAYER_RGGB8},
   {flir_fmt::BayerRG10p, "INV"},
   {flir_fmt::BayerRG10Packed, "INV"},
   {flir_fmt::BayerRG12p, "INV"},
   {flir_fmt::BayerRG12Packed, "INV"},
   {flir_fmt::BayerRG16, ros_fmt::BAYER_RGGB16},
   {flir_fmt::BayerGR8, ros_fmt::BAYER_GRBG8},
   {flir_fmt::BayerGR16, ros_fmt::BAYER_GRBG16},
   {flir_fmt::BayerGB8, ros_fmt::BAYER_GBRG8},
   {flir_fmt::BayerGB16, ros_fmt::BAYER_GBRG16},
   {flir_fmt::BayerBG8, ros_fmt::BAYER_BGGR8},
   {flir_fmt::BayerBG16, ros_fmt::BAYER_BGGR16},
   {flir_fmt::YUV411Packed, "INV"},
   {flir_fmt::YUV422Packed, "INV"},
   {flir_fmt::YUV444Packed, "INV"},
   {flir_fmt::YCbCr8, "INV"},
   {flir_fmt::YCbCr422_8, "INV"},
   {flir_fmt::YCbCr411_8, "INV"},
   {flir_fmt::RGB8, ros_fmt::RGB8},
   {flir_fmt::RGB8Packed, ros_fmt::RGB8},
   {flir_fmt::BGR8, ros_fmt::BGR8},
   {flir_fmt::BGRa8, ros_fmt::BGRA8}}};

static std::string flir_to_ros_encoding(const flir_fmt & pf, bool * canEncode)
{
  auto it = flir_2_ros.find(pf);
  *canEncode = (it != flir_2_ros.end() && it->second != "INV") && (pf != flir_fmt::INVALID);
  return (*canEncode ? it->second : "INV");
}

// adjust ROS header stamp using camera provided meta data
rclcpp::Time Camera::getAdjustedTimeStamp(uint64_t t, int64_t sensorTime)
{
  if (std::isnan(averageTimeDifference_)) {
    // capture the coarse offset between sensor and ROS time
    // at the very first time stamp.
    baseTimeOffset_ = static_cast<int64_t>(t) - sensorTime;
    averageTimeDifference_ = 0;
  }
  const double dt = (static_cast<int64_t>(t) - baseTimeOffset_ - sensorTime) * 1e-9;

  // compute exponential moving average
  constexpr double alpha = 0.01;  // average over rougly 100 samples
  averageTimeDifference_ = averageTimeDifference_ * (1.0 - alpha) + alpha * dt;

  // adjust sensor time by average difference to ROS time
  const rclcpp::Time adjustedTime = rclcpp::Time(sensorTime + baseTimeOffset_, RCL_SYSTEM_TIME) +
                                    rclcpp::Duration::from_seconds(averageTimeDifference_);
  return (adjustedTime);
}

void Camera::doPublish(const ImageConstPtr & im)
{
  rclcpp::Time t;
  if (synchronizer_) {
    uint64_t t_64;
    bool haveTime = synchronizer_->getTimeStamp(
      im->time_, im->imageTime_, im->frameId_, im->numIncomplete_, &t_64);
    t = rclcpp::Time(t_64, RCL_SYSTEM_TIME);
    if (!haveTime) {
      if (firstSynchronizedFrame_) {
        firstSynchronizedFrame_ = false;
      } else {
        LOG_WARN("cannot get time stamp for frame, dropping it");
      }
      return;
    }
  } else {
    t =
      adjustTimeStamp_ ? getAdjustedTimeStamp(im->time_, im->imageTime_) : rclcpp::Time(im->time_);
  }
  imageMsg_.header.frame_id = frameId_;
  imageMsg_.header.stamp = t;

  cameraInfoMsg_.header.frame_id = frameId_;
  cameraInfoMsg_.header.stamp = t;

  // Populating the messages

  if (imagePub_->get_subscription_count() > 0) {
    bool canEncode{false};
    const std::string encoding = flir_to_ros_encoding(im->pixelFormat_, &canEncode);
    if (!canEncode) {
      LOG_WARN(
        "no ROS encoding for pixel format "
        << spinnaker_camera_driver::pixel_format::to_string(im->pixelFormat_));
      return;
    }

    // will make deep copy. Do we need to? Probably...
    sensor_msgs::msg::Image::UniquePtr img(new sensor_msgs::msg::Image(imageMsg_));
    bool ret =
      sensor_msgs::fillImage(*img, encoding, im->height_, im->width_, im->stride_, im->data_);
    if (!ret) {
      LOG_ERROR("fill image failed!");
    } else {
      imagePub_->publish(std::move(img));
      // cameraInfoPub_->publish(std::move(cinfo));
      publishedCount_++;
    }
  }

  if (cameraInfoPub_->get_subscription_count() > 0) {
    cameraInfoPub_->publish(cameraInfoMsg_);
  }

  if (metaPub_->get_subscription_count() != 0) {
    metaMsg_.header.frame_id = frameId_;
    metaMsg_.header.stamp = t;
    metaMsg_.brightness = im->brightness_;
    metaMsg_.exposure_time = im->exposureTime_;
    metaMsg_.max_exposure_time = im->maxExposureTime_;
    metaMsg_.gain = im->gain_;
    metaMsg_.camera_time = im->imageTime_;
    metaPub_->publish(metaMsg_);
  }
}

void Camera::printCameraInfo()
{
  if (cameraRunning_) {
    LOG_INFO("camera has pixel format: " << wrapper_->getPixelFormat());
  }
}

void Camera::startCamera()
{
  if (!cameraRunning_) {
    spinnaker_camera_driver::SpinnakerWrapper::Callback cb =
      std::bind(&Camera::processImage, this, std::placeholders::_1);
    cameraRunning_ = wrapper_->startCamera(cb);
    if (!cameraRunning_) {
      LOG_ERROR("failed to start camera!");
    } else {
      printCameraInfo();
    }
  }
}

bool Camera::configure()
{
  readParameters();
  try {
    if (!readParameterDefinitionFile()) {
      return (false);
    }
  } catch (const YAML::Exception & e) {
    LOG_ERROR("error reading parameter definitions: " << e.what());
    return (false);
  }

  startWrapper();

  if (!initCamera()) {
    return (false);
  }

  if (!setCameraParams()) {
    return (false);
  }

  // Load CameraInfo
  if (!loadCameraInfo()) {
    return (false);
  }

  // Publishers
  // Creating replacement publishers to overcome not usable image_transport with lifeycle nodes
  metaPub_ = rclcpp::create_publisher<flir_camera_msgs::msg::ImageMetaData>(
    node_topics, "~/" + topicPrefix_ + "meta", rclcpp::QoS(1));
  imagePub_ = rclcpp::create_publisher<sensor_msgs::msg::Image>(
    node_topics, "~/" + topicPrefix_ + "image_raw", rclcpp::QoS(10));
  cameraInfoPub_ = rclcpp::create_publisher<sensor_msgs::msg::CameraInfo>(
    node_topics, "~/" + topicPrefix_ + "image_raw/camera_info", rclcpp::QoS(10));

  if (enableExternalControl_) {
    controlSub_ = rclcpp::create_subscription<flir_camera_msgs::msg::CameraControl>(
      node_topics, "~/" + topicPrefix_ + "control", 10,
      std::bind(&Camera::controlCallback, this, std::placeholders::_1));
  }

  return (true);
}

void Camera::startWrapper()
{
  wrapper_ = std::make_shared<spinnaker_camera_driver::SpinnakerWrapper>();
  wrapper_->getLibraryVersion();
  wrapper_->setDebug(debug_);
  wrapper_->setComputeBrightness(computeBrightness_);
  wrapper_->setAcquisitionTimeout(acquisitionTimeout_);
  LOG_INFO("wrapper initialized with version: " + wrapper_->getLibraryVersion());
}

bool Camera::initCamera()
{
  bool foundCamera = false;
  for (int retry = 1; retry < 6; retry++) {
    wrapper_->refreshCameraList();
    const auto camList = wrapper_->getSerialNumbers();
    if (std::find(camList.begin(), camList.end(), serial_) == camList.end()) {
      LOG_WARN("no camera found with serial: " << serial_ << " on try # " << retry);
      for (const auto & cam : camList) {
        LOG_WARN(" found cameras: " << cam);
      }
      std::this_thread::sleep_for(chrono::seconds(1));
    } else {
      LOG_INFO("found camera with serial number: " << serial_);
      foundCamera = true;
      break;
    }
  }
  if (!foundCamera) {
    LOG_ERROR("giving up, camera " << serial_ << " not found!");
    return (false);
  }
  return (true);
}

bool Camera::setCameraParams()
{
  if (wrapper_->initCamera(serial_)) {
    if (dumpNodeMap_) {
      LOG_INFO("dumping node map!");
      std::string nm = wrapper_->getNodeMapAsString();
      std::cout << nm;
    }
    // Must first create the camera parameters before acquisition is started.
    // Some parameters (like blackfly s chunk control) cannot be set once
    // the camera is running.
    createCameraParameters();

  } else {
    LOG_ERROR("init camera failed for cam: " << serial_);
    return (false);
  }
  return (true);
}

bool Camera::destroyComponents()
{
  // Cancel Timer
  if (statusTimer_ && !statusTimer_->is_canceled()) {
    statusTimer_->cancel();

    if (statusTimer_->is_canceled()) {
      LOG_INFO("statusTimer_ has been canceled.");
    } else {
      LOG_ERROR("statusTimer_ has not been canceled.");
      return false;
    }
  }

  if (checkSubscriptionsTimer_ && !checkSubscriptionsTimer_->is_canceled()) {
    checkSubscriptionsTimer_->cancel();

    if (checkSubscriptionsTimer_->is_canceled()) {
      LOG_INFO("checkSubscriptionsTimer_ has been canceled.");
    } else {
      LOG_ERROR("checkSubscriptionsTimer_ has not been canceled.");
      return false;
    }
  }

  // Destroy Publishers
  if (metaPub_) {
    metaPub_.reset();
    if (!metaPub_) {
      LOG_INFO("metaPub_ has been destroyed.");
    } else {
      LOG_ERROR("metaPub_ has not been destroyed.");
      return false;
    }
  }

  if (imagePub_) {
    imagePub_.reset();
    if (!imagePub_) {
      LOG_INFO("imagePub_ has been destroyed.");
    } else {
      LOG_ERROR("imagePub_ has not been destroyed.");
      return false;
    }
  }

  if (cameraInfoPub_) {
    cameraInfoPub_.reset();
    if (!cameraInfoPub_) {
      LOG_INFO("cameraInfoPub_ has been destroyed.");
    } else {
      LOG_ERROR("cameraInfoPub_ has not been destroyed.");
      return false;
    }
  }

  // Destroy Subscriptions
  if (controlSub_ && enableExternalControl_) {
    controlSub_.reset();
    if (!controlSub_) {
      LOG_INFO("controlSub_ has been destroyed.");
    } else {
      LOG_ERROR("controlSub_ has not been destroyed.");
      return false;
    }
  }

  // Reset Wrapper
  wrapper_.reset();

  return true;
}

bool Camera::loadCameraInfo()
{
  if (!cameraInfoURL_.empty()) {
    YAML::Node yamlFileCameraInfo = YAML::LoadFile(cameraInfoURL_);

    if (yamlFileCameraInfo.IsNull()) {
      LOG_ERROR("cannot open file: " << cameraInfoURL_);
      return (false);
    }

    cameraInfoMsg_.width = yamlFileCameraInfo["image_width"].as<int>();
    cameraInfoMsg_.height = yamlFileCameraInfo["image_height"].as<int>();
    cameraInfoMsg_.distortion_model = yamlFileCameraInfo["distortion_model"].as<std::string>();
    cameraInfoMsg_.d =
      yamlFileCameraInfo["distortion_coefficients"]["data"].as<std::vector<double>>();

    // Fill matrices K, R, P
    if (
      yamlFileCameraInfo["camera_matrix"]["data"] &&
      yamlFileCameraInfo["camera_matrix"]["data"].size() == 9) {
      for (std::size_t i = 0; i < 9; i++) {
        cameraInfoMsg_.k[i] = yamlFileCameraInfo["camera_matrix"]["data"][i].as<double>();
      }
    }

    if (
      yamlFileCameraInfo["rectification_matrix"]["data"] &&
      yamlFileCameraInfo["rectification_matrix"]["data"].size() == 9) {
      for (std::size_t i = 0; i < 9; i++) {
        cameraInfoMsg_.r[i] = yamlFileCameraInfo["rectification_matrix"]["data"][i].as<double>();
      }
    }

    if (
      yamlFileCameraInfo["projection_matrix"]["data"] &&
      yamlFileCameraInfo["projection_matrix"]["data"].size() == 12) {
      for (std::size_t i = 0; i < 12; i++) {
        cameraInfoMsg_.p[i] = yamlFileCameraInfo["projection_matrix"]["data"][i].as<double>();
      }
    }

    cameraInfoMsg_.binning_x = yamlFileCameraInfo["binning_x"].as<int>();
    cameraInfoMsg_.binning_y = yamlFileCameraInfo["binning_y"].as<int>();
    cameraInfoMsg_.roi.x_offset = yamlFileCameraInfo["roi"]["x_offset"].as<int>();
    cameraInfoMsg_.roi.y_offset = yamlFileCameraInfo["roi"]["y_offset"].as<int>();
    cameraInfoMsg_.roi.height = yamlFileCameraInfo["roi"]["height"].as<int>();
    cameraInfoMsg_.roi.width = yamlFileCameraInfo["roi"]["width"].as<int>();
    cameraInfoMsg_.roi.do_rectify = yamlFileCameraInfo["roi"]["do_rectify"].as<bool>();
  }

  LOG_INFO("camera info loaded from: " << cameraInfoURL_);
  return (true);
}

bool Camera::startAcquisition()
{
  keepRunning_ = true;
  thread_ = std::make_shared<std::thread>(&Camera::run, this);
  if (!connectWhileSubscribed_) {
    startCamera();
  } else {
    checkSubscriptionsTimer_ = rclcpp::create_timer(
      node_base, node_timer, node_clock->get_clock(), rclcpp::Duration(1, 0),
      std::bind(&Camera::checkSubscriptions, this));
  }
  return true;
}

bool Camera::stopAcquisition()
{
  stopCamera();
  keepRunning_ = false;
  if (thread_) {
    thread_->join();
    thread_ = 0;
  }

  return true;
}

bool Camera::start()
{
  if (!configure()) {
    return false;
  }

  if (!startAcquisition()) {
    return false;
  }

  return true;
}

template Camera::Camera(rclcpp::Node *, const std::string &, bool);
template Camera::Camera(rclcpp_lifecycle::LifecycleNode *, const std::string &, bool);

}  // namespace spinnaker_camera_driver
