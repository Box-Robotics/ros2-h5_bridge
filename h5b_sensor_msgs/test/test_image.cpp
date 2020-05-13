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
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");

  auto rw = [&h5](auto Tp, const std::string& path1, const std::string& path2,
                  int rows, int cols, int chans, const std::string& enc)
    {
      //
      // create some random image data and annotate it thusly
      //
      auto vec = h5_bridge::random_vec<decltype(Tp)>(rows*cols*chans);
      EXPECT_NO_THROW(h5->write(path1, vec, rows, cols, chans));
      h5->set_attr(path1, "header.frame_id", std::string(""));
      h5->set_attr(path1, "header.stamp.sec", std::int32_t{0});
      h5->set_attr(path1, "header.stamp.nanosec", std::uint32_t{0});
      h5->set_attr(path1, "encoding", enc);
      h5->set_attr(path1, "is_bigendian",
                   h5_bridge::big_endian() ? std::uint8_t{1} : std::uint8_t{0});

      //
      // read the data data out as a sensor_msgs::msg::Image
      //
      auto im = h5b::read<sensor_msgs::msg::Image>(h5.get(), path1);
      EXPECT_TRUE(im.height == rows);
      EXPECT_TRUE(im.width == cols);
      EXPECT_TRUE(im.encoding == enc);
      EXPECT_TRUE(im.step == cols*chans*sizeof(decltype(Tp)));

      //
      // write the data back as a sensor_msgs::msg::Image
      //
      EXPECT_NO_THROW(h5b::write(h5.get(), path2, im));

      //
      // read the data we just wrote back again and compare
      //
      auto im2 = h5b::read<sensor_msgs::msg::Image>(h5.get(), path2);
      EXPECT_TRUE(im == im2);
    };

  std::string path = "/sensor_msgs/msg/Image";
  int rows = 480;
  int cols = 640;

  rw(std::uint8_t{0}, path + "/8UC1/0", path + "/8UC1/1",
     rows, cols, 1, "8UC1");
  rw(std::uint8_t{0}, path + "/8UC2/0", path + "/8UC2/1",
     rows, cols, 2, "8UC2");
  rw(std::uint8_t{0}, path + "/8UC3/0", path + "/8UC3/1",
     rows, cols, 3, "8UC3");
  rw(std::uint8_t{0}, path + "/8UC4/0", path + "/8UC4/1",
     rows, cols, 4, "8UC4");

  rw(std::int8_t{0}, path + "/8SC1/0", path + "/8SC1/1",
     rows, cols, 1, "8SC1");
  rw(std::int8_t{0}, path + "/8SC2/0", path + "/8SC2/1",
     rows, cols, 2, "8SC2");
  rw(std::int8_t{0}, path + "/8SC3/0", path + "/8SC3/1",
     rows, cols, 3, "8SC3");
  rw(std::int8_t{0}, path + "/8SC4/0", path + "/8SC4/1",
     rows, cols, 4, "8SC4");

  rw(std::uint16_t{0}, path + "/16UC1/0", path + "/16UC1/1",
     rows, cols, 1, "16UC1");
  rw(std::uint16_t{0}, path + "/16UC2/0", path + "/16UC2/1",
     rows, cols, 2, "16UC2");
  rw(std::uint16_t{0}, path + "/16UC3/0", path + "/16UC3/1",
     rows, cols, 3, "16UC3");
  rw(std::uint16_t{0}, path + "/16UC4/0", path + "/16UC4/1",
     rows, cols, 4, "16UC4");

  rw(std::int16_t{0}, path + "/16SC1/0", path + "/16SC1/1",
     rows, cols, 1, "16SC1");
  rw(std::int16_t{0}, path + "/16SC2/0", path + "/16SC2/1",
     rows, cols, 2, "16SC2");
  rw(std::int16_t{0}, path + "/16SC3/0", path + "/16SC3/1",
     rows, cols, 3, "16SC3");
  rw(std::int16_t{0}, path + "/16SC4/0", path + "/16SC4/1",
     rows, cols, 4, "16SC4");

  rw(std::int32_t{0}, path + "/32SC1/0", path + "/32SC1/1",
     rows, cols, 1, "32SC1");
  rw(std::int32_t{0}, path + "/32SC2/0", path + "/32SC2/1",
     rows, cols, 2, "32SC2");
  rw(std::int32_t{0}, path + "/32SC3/0", path + "/32SC3/1",
     rows, cols, 3, "32SC3");
  rw(std::int32_t{0}, path + "/32SC4/0", path + "/32SC4/1",
     rows, cols, 4, "32SC4");

  rw(float{0}, path + "/32FC1/0", path + "/32FC1/1", rows, cols, 1, "32FC1");
  rw(float{0}, path + "/32FC2/0", path + "/32FC2/1", rows, cols, 2, "32FC2");
  rw(float{0}, path + "/32FC3/0", path + "/32FC3/1", rows, cols, 3, "32FC3");
  rw(float{0}, path + "/32FC4/0", path + "/32FC4/1", rows, cols, 4, "32FC4");

  rw(double{0}, path + "/64FC1/0", path + "/64FC1/1", rows, cols, 1, "64FC1");
  rw(double{0}, path + "/64FC2/0", path + "/64FC2/1", rows, cols, 2, "64FC2");
  rw(double{0}, path + "/64FC3/0", path + "/64FC3/1", rows, cols, 3, "64FC3");
  rw(double{0}, path + "/64FC4/0", path + "/64FC4/1", rows, cols, 4, "64FC4");
}
