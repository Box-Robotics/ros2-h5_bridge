#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include <h5_bridge.hpp>
#include <h5b_sensor_msgs/image.hpp>
#include <gtest/gtest.h>

namespace fs = std::filesystem;

auto h5_infile_ = []()->std::string
{
  auto tmp_dir = fs::temp_directory_path();
  return tmp_dir.native() + std::string("/h5b_sensor_msgs_test.h5");
};

const std::string H5_INFILE = h5_infile_();

TEST(image, Cleanup)
{
  EXPECT_NO_THROW(fs::remove(fs::path(H5_INFILE)));
}

TEST(image, ReadWrite)
{
  std::string path = "/sensor_msgs/msg/Image";
  int rows = 480;
  int cols = 640;
  int chans = 1;

  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");
  auto vec = h5_bridge::random_vec<std::uint8_t>(rows*cols*chans);
  EXPECT_NO_THROW(h5->write(path + "/u8/0", vec, rows, cols, chans));

  auto im = h5b::read<sensor_msgs::msg::Image>(h5.get(), path + "/u8/0");
  EXPECT_TRUE(vec == im.data);

  EXPECT_NO_THROW(h5b::write(h5.get(), path + "/u8/1", im));
  auto im2 = h5b::read<sensor_msgs::msg::Image>(h5.get(), path + "/u8/1");
  EXPECT_TRUE(im.data == im2.data);
}
