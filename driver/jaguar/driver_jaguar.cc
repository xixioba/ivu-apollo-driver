#include "driver_jaguar.h"

#include <regex>

#include "sdk/src/inno_lidar_api.h"
#include "sdk/src/inno_lidar_api_experimental.h"

namespace apollo {
namespace drivers {
namespace innovusion {

static void message_callback_s_(int handle_, void *ctx, enum inno_alarm level,
                                enum inno_alarm_code code, const char *msg) {
  DriverJaguar *context = reinterpret_cast<DriverJaguar *>(ctx);
  if (level >= (enum inno_alarm)context->inno_log_level)
    AWARN << "level=" << level << " code=" << code << " msg=" << msg;
  if (code == INNO_ALARM_CODE_CANNOT_READ &&
      (std::regex_match(std::string(msg), std::regex("^end of file.*")) ||
       context->data_filename != "")) {
    std::unique_lock<std::mutex> lk(context->mtx);
    context->is_running_ = -2;
    context->cv.notify_all();
  } else if (level >= INNO_ALARM_CRITICAL &&
             code != INNO_ALARM_CODE_LIB_VERSION_MISMATCH) {
    std::unique_lock<std::mutex> lk(context->mtx);
    context->is_running_ = -1;
    context->cv.notify_all();
  } else {
    std::unique_lock<std::mutex> lk(context->mtx);
    context->is_running_ = 1;
    context->cv.notify_all();
  }
}

bool DriverJaguar::init_() {
  // signal(SIGPIPE, SIG_IGN);
  // inno_lidar_setup_sig_handler();
  inno_lidar_set_log_level((enum inno_log_level)inno_log_level);
  if (data_filename != "") {
    if (yaml_filename == "") {
      AWARN << "Replay Jaguar Rawdata need yaml file";
      return false;
    }
    // setup read from file
    handle_ =
        inno_lidar_open_file(lidar_name.c_str(), data_filename.c_str(),
                             file_speed, file_rewind, file_skip * 1000000UL);
  } else {
    // setup read from file
    handle_ = inno_lidar_open_live(lidar_name.c_str(), lidar_ip.c_str(),
                                   lidar_port, true);
  }
  if (handle_ > 0) {
    inno_lidar_set_callbacks(handle_, message_callback_s_, NULL, NULL, this);
    inno_lidar_set_callbacks_2(handle_,
                               (inno_lidar_cframe_callback_t)data_callback_s_,
                               INNO_CFRAME_CPOINT, this);
    inno_lidar_set_parameters(handle_, lidar_model.c_str(), "",
                              yaml_filename.c_str());
    inno_lidar_set_reflectance_mode(handle_,
                                    (enum reflectance_mode)reflectance);
    return true;
  }
  return false;
};

bool DriverJaguar::start() {
  // no blocking mode
  if (handle_ <= 0) init_();
  inno_lidar_start(handle_);
  return true;
};

bool DriverJaguar::pause() {
  // no blocking mode
  if (handle_ > 0) {
    inno_lidar_stop(handle_);
  }
  return true;
};

bool DriverJaguar::stop() {
  if (handle_ > 0) {
    inno_lidar_stop(handle_);
    inno_lidar_close(handle_);
  }
  is_running_ = 0;
  handle_ = 0;
  return true;
};

int DriverJaguar::set_config_name_value(const std::string &key,
                                        const std::string &value) {
  return inno_lidar_set_config_name_value(handle_, key.c_str(), value.c_str());
}

}  // namespace innovusion
}  // namespace drivers
}  // namespace apollo
