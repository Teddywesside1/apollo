/*
 * @Author: binfeng.zhang
 * @Date: 2024-01-27 13:02:19
 * @LastEditors: binfeng.zhang
 * @LastEditTime: 2024-01-31 11:36:47
 * @Description: 
 */
#include "modules/zed/zed_component.h"

#include <cmath>
#include <string>

// #include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace zed {

bool ZedComponent::Init() {
  AINFO << "ZedComponent initializing ...";
  ZedComponentConfig zed_component_config;
  if (!GetProtoConfig(&zed_component_config)) {
    AERROR << "Load camera detection 3d component config failed!";
    return false;
  }
  // zed init
  if (!init_zed_device(zed_component_config.zed_config())){
    AERROR << "init zed device failed!";
    return false;
  }

  // cyber writer init
  if (!init_writer(zed_component_config)){
    AERROR << "init writer failed!";
    return false;
  }

  // start zed work thread
  try {
    _zed_work_thread = std::thread([this](ZedComponentConfig config){zed_work_thread_entry(config);}, 
                                  zed_component_config);
    _zed_work_thread.detach();    
  }catch (std::exception e){
    AERROR << "create zed work thread failed! " + std::string(e.what());
    return false;
  }

  AINFO << "ZedComponent init complete!";
  return true;
}


bool ZedComponent::init_zed_device(const ZedConfig& zed_config){
  auto zed_init_status = _zed_handle.init(zed_config);
  if (zed_init_status != ZED_ERROR_CODE::SUCCESS){
    return false;
  }
  return true;
}


bool ZedComponent::init_writer(const ZedComponentConfig &zed_component_config){
  _writer_left_image = node_->CreateWriter<apollo::drivers::Image>(
    zed_component_config.channel().left_image_channel_name()
  );

  _writer_right_image = node_->CreateWriter<apollo::drivers::Image>(
    zed_component_config.channel().right_image_channel_name()
  );

  return true;
}


void 
ZedComponent::zed_work_thread_entry(ZedComponentConfig config){
  AINFO << "zed work thread start !";
  int frame_id = 0;
  ZedFrameWrapper frame;
  while (_zed_handle.grab_one_frame(ZedGrabFrameConfig(), frame)
        == ZED_ERROR_CODE::SUCCESS){
    AINFO << frame_id << ", objects count : " << (frame.objects != nullptr ? frame.objects->object_list.size() : -1);
    ++ frame_id;
    cv::Mat left_image = *frame.left_image;
    
    std::shared_ptr<apollo::drivers::Image> out_msg_image
                          = std::make_shared<apollo::drivers::Image>();

    wrap_image_message(out_msg_image.get(), left_image);

    _writer_left_image->Write(out_msg_image);
  }

  AINFO << "end of zed work thread";
}


void 
ZedComponent::wrap_image_message(apollo::drivers::Image *img_msg, 
                                const cv::Mat &image){
  int rows = image.rows, cols = image.cols;
  int image_byte_size = rows * image.step;
  AWARN << "image.step : " << (int)image.step;
  img_msg->mutable_data()->reserve(image_byte_size);
  img_msg->set_data(image.data, image_byte_size);
  img_msg->set_height(rows);
  img_msg->set_width(cols);
  img_msg->set_encoding("rgb24");
  img_msg->mutable_header()->set_module_name("zed component");
  img_msg->set_measurement_time(cyber::Time::Now().ToNanosecond());
}


}  // namespace zed
}  // namespace apollo
