#pragma once

#include "cyber/cyber.h"
#include "driver_factory.h"
#include "modules/drivers/lidar/innovusion/proto/innovusion.pb.h"
#include "modules/drivers/lidar/innovusion/proto/innovusion_config.pb.h"
#include "modules/drivers/lidar/innovusion/proto/innovusion_imu.pb.h"

namespace apollo {
namespace drivers {
namespace innovusion {

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::innovusion::Imu;
using apollo::drivers::innovusion::PointCloud;
using apollo::drivers::innovusion::PointHVRIT;
using apollo::drivers::innovusion::PointXYZIT;
using apollo::drivers::innovusion::ScanCloud;

class InnovusionComponent : public apollo::cyber::Component<> {
 public:
  ~InnovusionComponent();
  bool Init() override;

  // static callback wrapper, called by the driver_
  static int data_callback_s_(int lidar_handle, void *ctx, void *frame) {
    return (reinterpret_cast<InnovusionComponent *>(ctx))
        ->data_callback_(frame);
  }
  // The callback actually
  int data_callback_(void *cframe);

  // static callback wrapper, called by the driver_
  static int status_callback_s_(int lidar_handle, void *ctx,
                                std::string status) {
    return (reinterpret_cast<InnovusionComponent *>(ctx))
        ->status_callback_(status);
  }
  // The callback actually
  int status_callback_(std::string status);

 protected:
  std::shared_ptr<DriverFactory> driver_ = nullptr;
  volatile int is_running_{0};  ///< device thread is running
  apollo::drivers::innovusion::Config conf_;
  std::shared_ptr<Writer<ScanCloud>> scan_writer_ = nullptr;
  std::shared_ptr<Writer<PointCloud>> pointcloud_writer_ = nullptr;
  std::shared_ptr<Writer<Imu>> imu_writer_ = nullptr;

  std::shared_ptr<PointCloud> point_cloud_ptr_ = nullptr;
  std::shared_ptr<ScanCloud> scan_cloud_ptr_ = nullptr;
  std::shared_ptr<Imu> imu_ptr_ = nullptr;
  uint32_t enable_fast_sin_cos{0};
};

CYBER_REGISTER_COMPONENT(InnovusionComponent)

}  // namespace innovusion
}  // namespace drivers
}  // namespace apollo
