/*
 * @Author: binfeng.zhang
 * @Date: 2024-01-28 12:48:43
 * @LastEditors: binfeng.zhang
 * @LastEditTime: 2024-01-31 11:40:08
 * @Description: 
 */
#include "modules/zed/zed_device.h"

// cyber
#include "cyber/common/log.h"

// zed

namespace apollo {
namespace zed {

ZED_ERROR_CODE 
ZedDevice::init(const ZedConfig config){
  // open and initialize zed device
  {
    sl::InitParameters init_param;
    set_zed_input(config, init_param);
    set_zed_resolution(config, init_param);
    set_zed_depth_mode(config, init_param);
    set_zed_coord(config, init_param);
    auto open_zed_state = _device.open(init_param);
    if (open_zed_state != sl::ERROR_CODE::SUCCESS){
      AERROR << "open zed device failed!";
      return ZED_ERROR_CODE::OPEN_FAILED;
    }
  }

  // config positional tracking function
  {
    sl::PositionalTrackingParameters positional_tracking_init_param;
    auto positional_tracking_config_state = _device.enablePositionalTracking(positional_tracking_init_param);
    if (positional_tracking_config_state != sl::ERROR_CODE::SUCCESS){
      AERROR << "config positional tracking function failed!";
      return ZED_ERROR_CODE::POSITION_TRACK_CONFIG_FAILED;
    }
  }

  // config object detection function
  {
    sl::ObjectDetectionParameters detection_init_param;
    detection_init_param.enable_tracking = true;
    detection_init_param.enable_segmentation = false;
    detection_init_param.detection_model = sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_FAST;
    auto detection_config_state = _device.enableObjectDetection(detection_init_param);
    if (detection_config_state != sl::ERROR_CODE::SUCCESS){
      AERROR << "config detection function failed!";
      return ZED_ERROR_CODE::DETECTION_CONFIG_FAILED;
    }
  }

  return ZED_ERROR_CODE::SUCCESS;
  //
}

ZED_ERROR_CODE 
ZedDevice::grab_one_frame(const ZedGrabFrameConfig &config, 
                          ZedFrameWrapper &frame){
  
  // grab one frame from zed device
  {  
    sl::RuntimeParameters runtime_param;
    runtime_param.confidence_threshold = config.detection_confidence_threshold;
    auto zed_grab_state = _device.grab(runtime_param);
    if (zed_grab_state != sl::ERROR_CODE::SUCCESS){
      AERROR << "zed grab frame failed!";
      return ZED_ERROR_CODE::GRAB_FAILED;
    }
  }

  // retrieve objects from frame data
  if (config.grab_objects){
    std::shared_ptr<sl::Objects> objects_ptr = std::make_shared<sl::Objects>();
    sl::ObjectDetectionRuntimeParameters objects_runtime_param;
    objects_runtime_param.object_class_filter = {sl::OBJECT_CLASS::PERSON, sl::OBJECT_CLASS::VEHICLE};
    objects_runtime_param.object_class_detection_confidence_threshold[sl::OBJECT_CLASS::PERSON] = config.detection_confidence_threshold;
    objects_runtime_param.object_class_detection_confidence_threshold[sl::OBJECT_CLASS::VEHICLE] = config.detection_confidence_threshold;
    auto retrieve_objects_state = _device.retrieveObjects(*objects_ptr, objects_runtime_param);
    if (retrieve_objects_state != sl::ERROR_CODE::SUCCESS){
      AERROR << "zed retrieve objects failed!";
      return ZED_ERROR_CODE::RETRIEVE_OBJECTS_FAILED;
    }
    frame.objects = objects_ptr;
  }

  // grab left image from frame data
  if (config.grab_left_image){
    auto camera_config = _device.getCameraInformation().camera_configuration;
    int image_width = camera_config.resolution.width;
    int image_height = camera_config.resolution.height;
    // single thread only
    static std::shared_ptr<cv::Mat> image_left_ptr = std::make_shared<cv::Mat>(image_height, image_width, CV_8UC4);
    // prepare memory buffer in opencv matrix
    if (image_left_ptr->rows != image_height || image_left_ptr->cols != image_width){
      image_left_ptr = std::make_shared<cv::Mat>(image_height, image_width, CV_8UC4);
    }

    // build a zed matrix handle to fill memory buffer
    sl::Resolution resolution = {image_width, image_height};
    sl::Mat image_retrieve_handle(resolution, sl::MAT_TYPE::U8_C4, 
                                  image_left_ptr->data, image_left_ptr->step);
    auto retrieve_image_state = _device.retrieveImage(image_retrieve_handle, sl::VIEW::LEFT, sl::MEM::CPU, resolution);
    if (retrieve_image_state != sl::ERROR_CODE::SUCCESS){
      AERROR << "zed retrieve left image failed!";
      return ZED_ERROR_CODE::RETRIEVE_IMAGE_FAILED;
    }
    std::shared_ptr<cv::Mat> image_out_ptr = std::make_shared<cv::Mat>();
    cv::cvtColor(*image_left_ptr, *image_out_ptr, cv::COLOR_BGRA2RGB);
    
    frame.left_image = image_out_ptr;
  }

  // grab right image from frame data
  if (config.grab_right_image){
    auto camera_config = _device.getCameraInformation().camera_configuration;
    int image_width = camera_config.resolution.width;
    int image_height = camera_config.resolution.height;
    // single thread only
    static std::shared_ptr<cv::Mat> image_right_ptr 
                      = std::make_shared<cv::Mat>(image_height, image_width, CV_8UC4);
    // prepare memory buffer in opencv matrix
    if (image_right_ptr->rows != image_height || image_right_ptr->cols != image_width){
      image_right_ptr = std::make_shared<cv::Mat>(image_height, image_width, CV_8UC4);
    }

    // build a zed matrix handle to fill memory buffer
    sl::Resolution resolution = {image_width, image_height};
    sl::Mat image_retrieve_handle(resolution, sl::MAT_TYPE::U8_C4, 
                                  image_right_ptr->data, image_right_ptr->step);
    auto retrieve_image_state = _device.retrieveImage(image_retrieve_handle, sl::VIEW::LEFT, sl::MEM::CPU, resolution);
    if (retrieve_image_state != sl::ERROR_CODE::SUCCESS){
      AERROR << "zed retrieve right image failed!";
      return ZED_ERROR_CODE::RETRIEVE_IMAGE_FAILED;
    }
    
    frame.right_image = image_right_ptr;
  }

  // grab cam pose
  if (config.grab_cam_pose){
    std::shared_ptr<sl::Pose> cam_pose_ptr = std::make_shared<sl::Pose>();
    auto retrieve_cam_pose_state = _device.getPosition(*cam_pose_ptr, sl::REFERENCE_FRAME::WORLD);
    if (retrieve_cam_pose_state == sl::POSITIONAL_TRACKING_STATE::OFF){
      AERROR << "zed retrieve cam pose failed!";
      return ZED_ERROR_CODE::RETRIEVE_CAM_POSE_FAILED;
    }

    frame.cam_pose = cam_pose_ptr;
  }

  return ZED_ERROR_CODE::SUCCESS;
}




void 
ZedDevice::set_zed_input(const ZedConfig& config, 
                        sl::InitParameters &init_param){
  if (config.has_input_svo_record()){
    init_param.input.setFromSVOFile(sl::String(config.input_svo_record().c_str()));
  }
}

void 
ZedDevice::set_zed_depth_mode(const ZedConfig& config, 
                              sl::InitParameters &init_param){
  init_param.depth_mode = sl::DEPTH_MODE::ULTRA;
  init_param.depth_maximum_distance = 10.f * 1000.f;
}

void 
ZedDevice::set_zed_resolution(const ZedConfig& config, 
                              sl::InitParameters &init_param){
  init_param.camera_resolution = sl::RESOLUTION::HD1080;
}

void 
ZedDevice::set_zed_coord(const ZedConfig& config, 
                        sl::InitParameters &init_param){
  init_param.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
}

} // zed
} // apollo

