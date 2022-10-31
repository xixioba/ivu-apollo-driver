#include "adapter_component.h"

#include <time.h>

#include <chrono>
#include <iostream>

#include "cyber/cyber.h"
#include "driver_factory.h"
#include "falcon/driver_falcon.h"
#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace innovusion {

class AdapterTest;
class Adapter : public InnovusionComponent {
 public:
  Adapter() {
    if (node_ == nullptr) {
      apollo::cyber::Init("AdapterTest");
      apollo::cyber::SetState(apollo::cyber::STATE_INITIALIZED);
      node_ = apollo::cyber::CreateNode("adapter");
    }
  }
  void load_config_files(std::string path) {
    apollo::cyber::proto::ComponentConfig cfg;
    cfg.set_config_file_path(path);
    LoadConfigFiles(cfg);
  }
  void set_lidar_model(std::string value) { conf_.set_lidar_model(value); };
  bool check_instance(std::string name) {
    return strstr(typeid(*driver_).name(), name.c_str()) ? true : false;
  }
  bool check_pointcloud_writer() {
    if (pointcloud_writer_) return true;
    return false;
  }
  bool check_scan_writer() {
    if (scan_writer_) return true;
    return false;
  }
  bool stop() { return driver_->stop(); }
  std::shared_ptr<PointCloud> get_pointcloud_ptr() { return point_cloud_ptr_; }
  std::shared_ptr<ScanCloud> get_scancloud_ptr() { return scan_cloud_ptr_; }

 private:
  friend class AdapterTest;
};

class AdapterTest : public testing::Test {
 public:
  AdapterTest(){};
  Adapter adapter;
};

TEST_F(AdapterTest, ModelFalcon) {
  adapter.load_config_files(
      "/apollo/modules/drivers/lidar/innovusion/conf/01.pb.txt");
  ASSERT_TRUE(adapter.Init());
  ASSERT_TRUE(adapter.stop());
  ASSERT_TRUE(adapter.check_instance("Falcon"));
}

TEST_F(AdapterTest, ModelJaguar) {
  adapter.load_config_files(
      "/apollo/modules/drivers/lidar/innovusion/conf/02.pb.txt");
  ASSERT_TRUE(adapter.Init());
  ASSERT_TRUE(adapter.stop());
  ASSERT_TRUE(adapter.check_instance("Jaguar"));
}

TEST_F(AdapterTest, ModelUnknown) {
  adapter.load_config_files(
      "/apollo/modules/drivers/lidar/innovusion/conf/03.pb.txt");
  ASSERT_FALSE(adapter.Init());
}

TEST_F(AdapterTest, DataPrase) {
  adapter.load_config_files(
      "/apollo/modules/drivers/lidar/innovusion/conf/01.pb.txt");
  ASSERT_TRUE(adapter.Init());
  ASSERT_TRUE(adapter.stop());
  ASSERT_TRUE(adapter.check_instance("Falcon"));
  ASSERT_TRUE(adapter.check_pointcloud_writer());
  ASSERT_TRUE(adapter.check_scan_writer());

  // create frame data
  srand((unsigned)time(NULL));
  int item_number = random() % 10 + 5;
  inno_cframe_header *frame = reinterpret_cast<inno_cframe_header *>(
      calloc(1, sizeof(struct inno_cframe_header) +
                    sizeof(struct inno_cpoint) * item_number));
  frame->item_number = item_number;
  frame->type = INNO_CFRAME_CPOINT;
  frame->ts_us_start =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now().time_since_epoch())
          .count() *
      1.0;
  frame->ts_us_end =
      (std::chrono::duration_cast<std::chrono::microseconds>(
           std::chrono::high_resolution_clock::now().time_since_epoch())
           .count() +
       100 * 1e3) *
      1.0;

  // feed point
  for (auto i = 0; i < item_number; i++) {
    inno_cpoint &p = frame->cpoints[i];
    p.h_angle = random() % 1000 - 500;
    p.v_angle = random() % 1000 - 500;
    p.ref = random() % 255;
    p.ts_100us = random() % 1000;
  }

  // parse data
  ASSERT_TRUE(item_number == adapter.data_callback_((void *)frame));
  auto point_cloud = adapter.get_pointcloud_ptr();
  auto scan_cloud = adapter.get_scancloud_ptr();
  ASSERT_TRUE(point_cloud);
  ASSERT_TRUE(scan_cloud);

  // verify frame data
  ASSERT_EQ(point_cloud->width(), item_number);
  ASSERT_EQ(point_cloud->width(), point_cloud->point_size());
  ASSERT_EQ(scan_cloud->width(), item_number);
  ASSERT_EQ(scan_cloud->width(), scan_cloud->point_size());

  uint64_t frame_ns_start_ref = (uint64_t)(frame->ts_us_start) * 1000;
  uint64_t frame_ns_end_ref = (uint64_t)(frame->ts_us_end) * 1000;
  std::cout << "frame time ref: " << frame_ns_start_ref
            << ", ts_us_start: " << point_cloud->frame_ns_start() << std::endl;
  std::cout << "frame end time ref: " << frame_ns_end_ref
            << ", ts_us_end: " << point_cloud->frame_ns_end() << std::endl;
  EXPECT_EQ(frame_ns_start_ref, point_cloud->frame_ns_start());
  EXPECT_EQ(frame_ns_end_ref, point_cloud->frame_ns_end());
  EXPECT_EQ(frame_ns_start_ref, scan_cloud->frame_ns_start());
  EXPECT_EQ(frame_ns_end_ref, scan_cloud->frame_ns_end());

  // verify point data
  for (auto i = 0; i < point_cloud->point_size(); ++i) {
    inno_cpoint &p = frame->cpoints[i];
    uint64_t ref_point_ns = frame_ns_start_ref + (uint64_t)(p.ts_100us * 1e5);
    uint64_t real_point_ns = point_cloud->point(i).timestamp();
    std::cout << "point[" << i << "] time " << ref_point_ns << " "
              << real_point_ns << std::endl;
    EXPECT_EQ(ref_point_ns, real_point_ns);
  }

  free(frame);
}

// live reconnect test -> falcon
// has been manually tested 10 times -> OK
// live reconnect test -> Jaguar
// has been manually tested 10 times -> OK

// rawdata read test -> Falcon
//  has been manually tested -> OK
// rawdata read test -> Jaguar
//  has been manually tested -> OK

// TODO config test

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  testing::FLAGS_gtest_death_test_style = "fast";
  testing::FLAGS_gtest_death_test_style = "threadsafe";
  return RUN_ALL_TESTS();
}

}  // namespace innovusion
}  // namespace drivers
}  // namespace apollo
