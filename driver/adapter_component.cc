
#include "adapter_component.h"

#include <regex>

#include "falcon/driver_falcon.h"
#include "jaguar/driver_jaguar.h"
#include "nlohmann/json.hpp"

namespace apollo {
namespace drivers {
namespace innovusion {

using json = nlohmann::json;

// sine cosine acceleration
// comparison: fastest implementation vs look up table(lut, 100 values per
// radian) vs std max error : 0.002% vs 0.001% vs 0% time consumption: 0.13 vs
// 0.25 vs 0.35

// double abs fast implementation
double absd(double a) {
  *((unsigned long *)&a) &= ~(1UL << 63);
  return a;
}

// input limit: -pi ~ pi
double fast_sine(double x) {
  double y = x * (1.273239545 + -0.405284735 * absd(x));
  return y * (absd(y) * (0.0192 * absd(y) + 0.1951) + 0.7857);
}

// input limit: -pi-M_PI_2 ~ pi-M_PI_2
double fast_cosine(double x) { return fast_sine(x + M_PI_2); }

int InnovusionComponent::data_callback_(void *cframe) {
  inno_cframe_header *frame = (inno_cframe_header *)cframe;
  // process full frame
  if (frame && (scan_writer_ || pointcloud_writer_)) {
    // only use INNO_CFRAME_CPOINT
    if (frame->type == INNO_CFRAME_CPOINT) {
      // check time shifting
      if (driver_->time_fix_err_ms != 0) {
        uint64_t local_ts_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count();
        if (driver_->time_fix_err_ms < 0 ||
            abs((int64_t)local_ts_ns - (int64_t)(frame->ts_us_start) * 1000) >
                driver_->time_fix_err_ms * 1e6) {
          AWARN << "Driver Detect frame[" << frame->idx % UINT_MAX
                << "] shifting big than " << driver_->time_fix_err_ms
                << " move " << (uint64_t)(frame->ts_us_start) * 1000 << " to "
                << local_ts_ns;
          frame->ts_us_start = local_ts_ns / 1e9;
          frame->ts_us_start = (local_ts_ns + 100 * 1e6) / 1e9;
        }
      }
      if (scan_writer_) {
        // reset data
        scan_cloud_ptr_.reset(new ScanCloud);
        // set header
        scan_cloud_ptr_->mutable_header()->set_frame_id(conf_.frame_id());
        scan_cloud_ptr_->mutable_header()->set_sequence_num(frame->idx %
                                                            UINT_MAX);
        scan_cloud_ptr_->mutable_header()->set_lidar_timestamp(
            (uint64_t)(frame->ts_us_start) * 1000);
        // set frame param
        scan_cloud_ptr_->set_frame_id(conf_.frame_id());
        scan_cloud_ptr_->set_idx(frame->idx);
        scan_cloud_ptr_->set_measurement_time(frame->ts_us_start * 1e-6);
        scan_cloud_ptr_->set_frame_ns_start((uint64_t)(frame->ts_us_start) *
                                            1000);
        scan_cloud_ptr_->set_frame_ns_end((uint64_t)(frame->ts_us_end) * 1000);
        scan_cloud_ptr_->set_model(conf_.lidar_model());
        scan_cloud_ptr_->set_source_id(conf_.lidar_id());
        scan_cloud_ptr_->set_height(1);
        scan_cloud_ptr_->set_width(frame->item_number);
      }
      if (pointcloud_writer_) {
        // reset data
        point_cloud_ptr_.reset(new PointCloud);
        // set header
        point_cloud_ptr_->mutable_header()->set_frame_id(conf_.frame_id());
        point_cloud_ptr_->mutable_header()->set_sequence_num(frame->idx %
                                                             UINT_MAX);
        point_cloud_ptr_->mutable_header()->set_lidar_timestamp(
            ((uint64_t)(frame->ts_us_start)) * 1000);
        // set frame param
        point_cloud_ptr_->set_frame_id(conf_.frame_id());
        point_cloud_ptr_->set_idx(frame->idx);
        point_cloud_ptr_->set_measurement_time(frame->ts_us_start * 1e-6);
        point_cloud_ptr_->set_frame_ns_start((uint64_t)(frame->ts_us_start) *
                                             1000);
        point_cloud_ptr_->set_frame_ns_end((uint64_t)(frame->ts_us_end) * 1000);
        point_cloud_ptr_->set_model(conf_.lidar_model());
        point_cloud_ptr_->set_source_id(conf_.lidar_id());
        point_cloud_ptr_->set_height(1);
        point_cloud_ptr_->set_width(frame->item_number);
      }
      // get every point from frame
      for (unsigned int i = 0; i < frame->item_number; i++) {
        if (frame->type == INNO_CFRAME_CPOINT) {
          inno_cpoint *p = &frame->cpoints[i];
          if (std::isnan(p->h_angle) || std::isnan(p->v_angle) ||
              std::isnan(p->radius))
            continue;
          // fill scancloud
          if (scan_writer_) {
            PointHVRIT *point = scan_cloud_ptr_->add_point();
            point->set_h_angle(p->h_angle);
            point->set_v_angle(p->v_angle);
            point->set_radius(p->radius);  // in cm uint
            point->set_timestamp((uint64_t)(p->ts_100us * 1e5) +
                                 (uint64_t)(frame->ts_us_start) * 1000);
            point->set_intensity(static_cast<uint>(p->ref));
            point->set_flags(p->flags);
            point->set_scan_id(p->scan_id);
            point->set_scan_idx(p->scan_idx);
          }
          // fill pointcloud
          if (pointcloud_writer_) {
            PointXYZIT *point = point_cloud_ptr_->add_point();
            double radius = p->radius / 100.0;
            double x = 0.0, y = 0.0, t = 0.0, z = 0.0;
            if (enable_fast_sin_cos == 0) {
              x = radius * sin(p->v_angle * cpoint_angle_unit_c);
              t = radius * cos(p->v_angle * cpoint_angle_unit_c);
              y = t * sin(p->h_angle * cpoint_angle_unit_c);
              z = t * cos(p->h_angle * cpoint_angle_unit_c);
            } else if (enable_fast_sin_cos == 1) {
              x = radius * fast_sine(p->v_angle * cpoint_angle_unit_c);
              t = radius * fast_cosine(p->v_angle * cpoint_angle_unit_c);
              y = t * fast_sine(p->h_angle * cpoint_angle_unit_c);
              z = t * fast_cosine(p->h_angle * cpoint_angle_unit_c);
            }
            point->set_x(x);
            point->set_y(y);
            point->set_z(z);
            point->set_timestamp((uint64_t)(p->ts_100us * 1e5) +
                                 (uint64_t)(frame->ts_us_start) * 1000);
            point->set_intensity(static_cast<uint>(p->ref));
            point->set_flags(p->flags);
            point->set_scan_id(p->scan_id);
            point->set_scan_idx(p->scan_idx);
          }
        }
      }

      // write channel
      if (scan_writer_ && scan_cloud_ptr_) {
        scan_cloud_ptr_->mutable_header()->set_timestamp_sec(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count() /
            1e9);
        scan_writer_->Write(scan_cloud_ptr_);
      }
      if (pointcloud_writer_ && point_cloud_ptr_) {
        point_cloud_ptr_->mutable_header()->set_timestamp_sec(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count() /
            1e9);
        pointcloud_writer_->Write(point_cloud_ptr_);
      }
    }
  }
  return frame->item_number;
}

int InnovusionComponent::status_callback_(std::string status) {
  try {
    auto j = json::parse(status);
    if (!j.empty() && j.contains("get-gyroscope-xyz")) {
      if (imu_writer_) {
        imu_ptr_.reset(new Imu);
        // int accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z;
        imu_ptr_->set_accel_x(j["get-gyroscope-xyz"]["accel_x"].get<int>());
        imu_ptr_->set_accel_y(j["get-gyroscope-xyz"]["accel_y"].get<int>());
        imu_ptr_->set_accel_z(j["get-gyroscope-xyz"]["accel_z"].get<int>());
        imu_ptr_->set_gyro_x(j["get-gyroscope-xyz"]["gyro_x"].get<int>());
        imu_ptr_->set_gyro_y(j["get-gyroscope-xyz"]["gyro_y"].get<int>());
        imu_ptr_->set_gyro_z(j["get-gyroscope-xyz"]["gyro_z"].get<int>());
        // imu_ptr_->set_temperature(std::stof(j["get-gyroscope-xyz"]["temperature"].get<std::string>()));
        imu_ptr_->mutable_header()->set_timestamp_sec(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count() /
            1e9);
        imu_ptr_->set_measurement_time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count() /
            1e9);
        imu_writer_->Write(imu_ptr_);
      }
    }
  } catch (const json::exception &e) {
    // TODO At present, there is a problem with the firmware information format,
    // and the json cannot be successfully parsed, so the exception is
    // temporarily shielded
    // std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
    //           << __FUNCTION__ << std ::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
              << __FUNCTION__ << std ::endl;
  } catch (...) {
    std ::cerr << "unknown exception"
               << " " << __FILE__ << " " << __LINE__ << " " << __FUNCTION__
               << std ::endl;
  }
  return 0;
};

InnovusionComponent::~InnovusionComponent(){};

bool InnovusionComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    return false;
  }

  signal(SIGINT, [](int sign) {
    if (sign == SIGINT) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      pid_t pid = getpid();
      AWARN << "kill driver pid:" << pid;
      kill(pid, SIGKILL);
    }
  });

  apollo::cyber::binary::SetName(node_->Name());

  ADEBUG << "get config:\n" << conf_.DebugString();

  if (conf_.has_pointcloud_channel() && conf_.pointcloud_channel() != "" &&
      node_) {
    ADEBUG << "create pointcloud_channel " << conf_.pointcloud_channel();
    pointcloud_writer_ =
        node_->CreateWriter<PointCloud>(conf_.pointcloud_channel());
  }
  if (conf_.has_scan_channel() && conf_.scan_channel() != "" && node_) {
    ADEBUG << "create scan_channel " << conf_.scan_channel();
    scan_writer_ = node_->CreateWriter<ScanCloud>(conf_.scan_channel());
  }
  if (conf_.has_imu_channel() && conf_.imu_channel() != "" && node_) {
    ADEBUG << "create imu_channel " << conf_.imu_channel();
    imu_writer_ = node_->CreateWriter<Imu>(conf_.imu_channel());
  }

  // lowercase
  std::string lidar_model = conf_.lidar_model();
  for (size_t i = 0; i < lidar_model.length(); ++i)
    lidar_model[i] = std::tolower(lidar_model[i]);

  ADEBUG << "Get innovusion model: " << lidar_model;
  // choose model
  if (std::regex_match(lidar_model, std::regex("rev_[i,f,b,r,k].*"))) {
    AWARN << "Init InnovusionDriverFalcon";
    driver_.reset(new DriverFalcon(data_callback_s_, this, status_callback_s_));
  } else if (std::regex_match(lidar_model, std::regex("rev_[g,e,h].*"))) {
    AWARN << "Init InnovusionDriverJaguar";
    driver_.reset(new DriverJaguar(data_callback_s_, this));
  } else {
    AERROR << "Not support model:" << conf_.lidar_model();
    return false;
  }

  if (conf_.has_lidar_name()) driver_->lidar_name = conf_.lidar_name();
  if (conf_.has_frame_id()) driver_->frame_id = conf_.frame_id();
  if (conf_.has_lidar_id()) driver_->lidar_id = conf_.lidar_id();
  if (conf_.has_lidar_ip()) driver_->lidar_ip = conf_.lidar_ip();
  if (conf_.has_lidar_port()) driver_->lidar_port = conf_.lidar_port();
  if (conf_.has_lidar_model()) driver_->lidar_model = conf_.lidar_model();
  if (conf_.has_reflectance()) driver_->reflectance = conf_.reflectance();
  if (conf_.has_multireturn()) driver_->multireturn = conf_.multireturn();
  if (conf_.has_data_filename()) driver_->data_filename = conf_.data_filename();
  if (conf_.has_yaml_filename()) driver_->yaml_filename = conf_.yaml_filename();
  if (conf_.has_file_speed()) driver_->file_speed = conf_.file_speed();
  if (conf_.has_file_rewind()) driver_->file_rewind = conf_.file_rewind();
  if (conf_.has_file_skip()) driver_->file_skip = conf_.file_skip();
  if (conf_.has_lidar_udp_port())
    driver_->lidar_udp_port = conf_.lidar_udp_port();
  if (conf_.has_processed()) driver_->processed = conf_.processed();
  if (conf_.has_set_falcon_eye())
    driver_->set_falcon_eye = conf_.set_falcon_eye();
  if (conf_.has_roi_center_h()) driver_->roi_center_h = conf_.roi_center_h();
  if (conf_.has_roi_center_v()) driver_->roi_center_v = conf_.roi_center_v();
  if (conf_.has_inno_log_level())
    driver_->inno_log_level = conf_.inno_log_level();
  if (conf_.has_time_fix_err_ms())
    driver_->time_fix_err_ms = conf_.time_fix_err_ms();
  if (conf_.has_enable_fast_sin_cos())
    enable_fast_sin_cos = conf_.enable_fast_sin_cos();
  driver_->start();
  return true;
};
}  // namespace innovusion
}  // namespace drivers
}  // namespace apollo