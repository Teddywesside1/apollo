/*
 * @Author: binfeng.zhang
 * @Date: 2024-01-28 10:01:19
 * @LastEditors: binfeng.zhang
 * @LastEditTime: 2024-01-31 11:28:36
 * @Description: 
 */
#pragma once

//threads
#include <thread>

//cyber
#include "cyber/cyber.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"

// zed
#include "zed_device.h"

namespace apollo {
namespace zed{


class ZedComponent final 
  : public cyber::Component<> {
public:
  ZedComponent() {}
  ~ZedComponent() = default;

  bool Init() override;

private:
  bool init_zed_device(const ZedConfig& zed_config);
  bool init_writer(const ZedComponentConfig &zed_component_config);

  void zed_work_thread_entry(ZedComponentConfig config);

  // wrap message as protobuf
  void wrap_image_message(apollo::drivers::Image *img_msg, const cv::Mat &image);

private:
  std::thread _zed_work_thread;
  ZedDevice _zed_handle;

  std::shared_ptr<cyber::Writer<apollo::drivers::Image>> _writer_left_image;
  std::shared_ptr<cyber::Writer<apollo::drivers::Image>> _writer_right_image;
  // std::shared_ptr<cyber::Writer<apollo::> _writer_left_image;
};


CYBER_REGISTER_COMPONENT(ZedComponent);

} // zed
} // apollo



