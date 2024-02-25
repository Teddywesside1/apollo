/*
 * @Author: binfeng.zhang
 * @Date: 2024-01-28 12:48:49
 * @LastEditors: binfeng.zhang
 * @LastEditTime: 2024-01-31 03:06:58
 * @Description: 
 */
#pragma once

//cyber
#include "modules/zed/proto/zed_component.pb.h"

// zed
#include <sl/Camera.hpp>

// opencv
#include <opencv2/opencv.hpp>


namespace apollo {
namespace zed{

enum ZED_ERROR_CODE{
  SUCCESS,
  OPEN_FAILED,
  POSITION_TRACK_CONFIG_FAILED,
  DETECTION_CONFIG_FAILED,
  GRAB_FAILED,
  RETRIEVE_OBJECTS_FAILED,
  RETRIEVE_IMAGE_FAILED,
  RETRIEVE_CAM_POSE_FAILED,
};

/**
 * @description: configuration of zed frame grabing process
 */
struct ZedGrabFrameConfig {
  bool grab_left_image {true};
  bool grab_right_image {false};
  bool grab_point_cloud {false};
  bool grab_objects {true};
  bool grab_cam_pose {true};

  float detection_confidence_threshold {60.f};
};

struct ZedFrameWrapper{
  std::shared_ptr<cv::Mat> left_image {nullptr};
  std::shared_ptr<cv::Mat> right_image {nullptr}; // not used
  std::shared_ptr<sl::Objects> objects {nullptr};
  std::shared_ptr<sl::Pose> cam_pose {nullptr};
};

class ZedDevice {
public:
  ZedDevice() = default;
  
  ZED_ERROR_CODE init(const ZedConfig config);

  /**
   * @description: grab frame data from zed, 
   *                should be called after 'init' func
   * @param {ZedGrabFrameConfig} &config
   * @param {ZedFrameWrapper} frame
   * @return {*}
   */
  ZED_ERROR_CODE grab_one_frame(const ZedGrabFrameConfig &config, ZedFrameWrapper &frame);


private:
  /**
   * @description: 
   * @param {ZedConfig} &config
   * @param {InitParameters} &init_param
   * @return {*}
   */
  void set_zed_input(const ZedConfig &config, sl::InitParameters &init_param);
  /**
   * @description: 
   * @param {ZedConfig} &config
   * @param {InitParameters} &init_param
   * @return {*}
   */
  void set_zed_resolution(const ZedConfig &config, sl::InitParameters &init_param);
  /**
   * @description: 
   * @param {ZedConfig} &config
   * @param {InitParameters} &init_param
   * @return {*}
   */
  void set_zed_depth_mode(const ZedConfig &config, sl::InitParameters &init_param);
  /**
   * @description: 
   * @param {ZedConfig} &config
   * @param {InitParameters} &init_param
   * @return {*}
   */
  void set_zed_coord(const ZedConfig &config, sl::InitParameters &init_param);

private:
  sl::Camera _device {};
};

} // zed
} // apollo