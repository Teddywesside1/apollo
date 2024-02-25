

#pragma once 

// threads
#include <thread>

// cyber
#include "cyber/cyber.h"

// zed
#include <sl/Camera.hpp>

namespace apollo {
namespace zed {

class ZedComponent final
    : public cyber::Component<> {
public:
    ZedComponent() {}
  ~ZedComponent() = default;

  bool Init() override;

 private:
 std::thread _zed_work_thread {};

};

CYBER_REGISTER_COMPONENT(ZedComponent);

}  // namespace zed
}  // namespace apollo


