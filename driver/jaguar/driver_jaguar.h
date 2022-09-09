#pragma once

#include "modules/drivers/lidar/innovusion/driver/driver_factory.h"

namespace apollo {
namespace drivers {
namespace innovusion {

class __attribute__((visibility("default"))) DriverJaguar
    : public DriverFactory {
 public:
  DriverJaguar(InnoCframeCallBack data_callback, void *callback_context,
               InnoStatusCallBack status_callback = nullptr)
      : DriverFactory(data_callback, callback_context, status_callback){};
  ~DriverJaguar() {
    stop();  // make sure that handle_ has been closed
  };

  // callback group
  int data_callback_(int handle_, void *ctx, void *cframe) {
    if (cframe != NULL) {
      cframe_callback_(handle_, cframe_callback_ctx_, cframe);
    }
    return 0;
  };
  // static callback wrapper
  static int data_callback_s_(int handle_, void *ctx, void *cframe) {
    return (reinterpret_cast<DriverJaguar *>(ctx))
        ->data_callback_(handle_, ctx, cframe);
  }

  // lidar configuration
  bool init_();
  bool start() override;
  bool pause() override;
  bool stop() override;

  // optional
  int get_lidar(const std::string &cmd, std::string *result) override {
    return 0;
  };
  int set_lidar(const std::string &key, const std::string &value) override {
    return 0;
  };
  int set_config_name_value(const std::string &key,
                            const std::string &value) override;
};

}  // namespace innovusion
}  // namespace drivers
}  // namespace apollo
