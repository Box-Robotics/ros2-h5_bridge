#include <algorithm>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <opencv2/core.hpp>
#include <h5_bridge/h5_file.hpp>
#include <h5b_opencv.hpp>
#include <gtest/gtest.h>

namespace fs = std::filesystem;

auto h5_infile_ = []()->std::string
{
  auto tmp_dir = fs::temp_directory_path();
  return tmp_dir.native() + std::string("/h5b_opencv_test.h5");
};

const std::string H5_INFILE = h5_infile_();

TEST(OpenCV, Cleanup)
{
  EXPECT_NO_THROW(fs::remove(fs::path(H5_INFILE)));
}

TEST(Opencv, TypeErased)
{
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");

  auto rw = [&h5](auto Tp, const std::string& path1, const std::string& path2,
                  int rows, int cols, int cv_type)
    {
      //
      // Create a cv::Mat of random data and write it to H5
      //
      cv::Mat mat = cv::Mat::zeros(rows, cols, cv_type);

      auto vec =
        h5_bridge::random_vec<decltype(Tp)>(rows*cols*mat.channels());
      std::memcpy(mat.data, vec.data(), vec.size()*sizeof(decltype(Tp)));

      EXPECT_NO_THROW(h5b_opencv::write(h5.get(), path1, mat));

      //
      // Read what we just wrote back out to a new cv::Mat
      //
      cv::Mat mat2;
      EXPECT_NO_THROW(mat2 = h5b_opencv::read(h5.get(), path1));
      EXPECT_EQ(mat2.rows, rows);
      EXPECT_EQ(mat2.cols, cols);
      EXPECT_EQ(mat2.type(), cv_type);

      std::vector<decltype(Tp)> vec2(rows*cols*mat2.channels());
      std::memcpy(&vec2[0], mat2.data, vec2.size()*sizeof(decltype(Tp)));
      EXPECT_TRUE(vec == vec2);

      //
      // Write again
      //
      EXPECT_NO_THROW(h5b_opencv::write(h5.get(), path2, mat2));

      //
      // Read again, and compare
      //
      cv::Mat mat3;
      EXPECT_NO_THROW(mat3 = h5b_opencv::read(h5.get(), path2));
      EXPECT_EQ(mat3.rows, rows);
      EXPECT_EQ(mat3.cols, cols);
      EXPECT_EQ(mat3.type(), cv_type);

      std::vector<decltype(Tp)> vec3(rows*cols*mat3.channels());
      std::memcpy(&vec3[0], mat3.data, vec3.size()*sizeof(decltype(Tp)));
      EXPECT_TRUE(vec == vec3);
    };

  std::string path = "/opencv/type_erased";
  int rows = 480;
  int cols = 640;

  rw(std::uint8_t{0}, path + "/8UC1/0", path + "/8UC1/1", rows, cols, CV_8UC1);
  rw(std::uint8_t{0}, path + "/8UC2/0", path + "/8UC2/1", rows, cols, CV_8UC2);
  rw(std::uint8_t{0}, path + "/8UC3/0", path + "/8UC3/1", rows, cols, CV_8UC3);
  rw(std::uint8_t{0}, path + "/8UC4/0", path + "/8UC4/1", rows, cols, CV_8UC4);

  rw(std::int8_t{0}, path + "/8SC1/0", path + "/8SC1/1", rows, cols, CV_8SC1);
  rw(std::int8_t{0}, path + "/8SC2/0", path + "/8SC2/1", rows, cols, CV_8SC2);
  rw(std::int8_t{0}, path + "/8SC3/0", path + "/8SC3/1", rows, cols, CV_8SC3);
  rw(std::int8_t{0}, path + "/8SC4/0", path + "/8SC4/1", rows, cols, CV_8SC4);

  rw(std::uint16_t{0}, path + "/16UC1/0", path + "/16UC1/1",
     rows, cols, CV_16UC1);
  rw(std::uint16_t{0}, path + "/16UC2/0", path + "/16UC2/1",
     rows, cols, CV_16UC2);
  rw(std::uint16_t{0}, path + "/16UC3/0", path + "/16UC3/1",
     rows, cols, CV_16UC3);
  rw(std::uint16_t{0}, path + "/16UC4/0", path + "/16UC4/1",
     rows, cols, CV_16UC4);

  rw(std::int16_t{0}, path + "/16SC1/0", path + "/16SC1/1",
     rows, cols, CV_16SC1);
  rw(std::int16_t{0}, path + "/16SC2/0", path + "/16SC2/1",
     rows, cols, CV_16SC2);
  rw(std::int16_t{0}, path + "/16SC3/0", path + "/16SC3/1",
     rows, cols, CV_16SC3);
  rw(std::int16_t{0}, path + "/16SC4/0", path + "/16SC4/1",
     rows, cols, CV_16SC4);

  rw(std::int32_t{0}, path + "/32SC1/0", path + "/32SC1/1",
     rows, cols, CV_32SC1);
  rw(std::int32_t{0}, path + "/32SC2/0", path + "/32SC2/1",
     rows, cols, CV_32SC2);
  rw(std::int32_t{0}, path + "/32SC3/0", path + "/32SC3/1",
     rows, cols, CV_32SC3);
  rw(std::int32_t{0}, path + "/32SC4/0", path + "/32SC4/1",
     rows, cols, CV_32SC4);

  rw(float{0}, path + "/32FC1/0", path + "/32FC1/1", rows, cols, CV_32FC1);
  rw(float{0}, path + "/32FC2/0", path + "/32FC2/1", rows, cols, CV_32FC2);
  rw(float{0}, path + "/32FC3/0", path + "/32FC3/1", rows, cols, CV_32FC3);
  rw(float{0}, path + "/32FC4/0", path + "/32FC4/1", rows, cols, CV_32FC4);

  rw(double{0}, path + "/64FC1/0", path + "/64FC1/1", rows, cols, CV_64FC1);
  rw(double{0}, path + "/64FC2/0", path + "/64FC2/1", rows, cols, CV_64FC2);
  rw(double{0}, path + "/64FC3/0", path + "/64FC3/1", rows, cols, CV_64FC3);
  rw(double{0}, path + "/64FC4/0", path + "/64FC4/1", rows, cols, CV_64FC4);
}

TEST(Opencv, StronglyTyped)
{
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");

  auto rw = [&h5](auto Tp, const std::string& path, int rows, int cols)
    {
      auto vec = h5_bridge::random_vec<decltype(Tp)>(rows*cols);
      cv::Mat_<decltype(Tp)> mat = cv::Mat_<decltype(Tp)>::zeros(rows, cols);
      std::copy(vec.begin(), vec.end(), mat.begin());
      EXPECT_NO_THROW(h5b_opencv::write(h5.get(), path, mat));

      cv::Mat mat2;
      EXPECT_NO_THROW(mat2 = h5b_opencv::read(h5.get(), path));
      EXPECT_TRUE(std::equal(mat.begin(), mat.end(),
                             mat2.begin<decltype(Tp)>()));
    };

  std::string path = "/opencv/strongly_typed";
  int rows = 480;
  int cols = 640;

  rw(std::uint8_t{0}, path + "/8UC1/0", rows, cols);
  rw(std::int8_t{0}, path + "/8SC1/0", rows, cols);
  rw(std::uint16_t{0}, path + "/16UC1/0", rows, cols);
  rw(std::int16_t{0}, path + "/16SC1/0", rows, cols);
  rw(std::int32_t{0}, path + "/32SC1/0", rows, cols);
  rw(float{0}, path + "/32FC1/0", rows, cols);
  rw(double{0}, path + "/64FC1/0", rows, cols);
}
