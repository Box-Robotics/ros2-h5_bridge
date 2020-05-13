#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <h5_bridge.hpp>
#include <h5b_sensor_msgs.hpp>
#include <gtest/gtest.h>

namespace fs = std::filesystem;

auto h5_infile_ = []()->std::string
{
  auto tmp_dir = fs::temp_directory_path();
  return tmp_dir.native() + std::string("/h5b_sensor_msgs_pcl_test.h5");
};

const std::string H5_INFILE = h5_infile_();

struct EIGEN_ALIGN16 PointOS
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  std::uint8_t ring;
  std::uint16_t noise;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static inline PointOS make(
      float x, float y, float z, float intensity,
      std::uint32_t t, std::uint16_t reflectivity,
      std::uint8_t ring, std::uint8_t col,
      std::uint16_t noise, std::uint32_t range)
  {
    return {x, y, z, 0.0, intensity, t, reflectivity, ring, noise, range};
  }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointOS,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint32_t, t, t)
  (std::uint16_t, reflectivity, reflectivity)
  (std::uint8_t, ring, ring)
  (std::uint16_t, noise, noise)
  (std::uint32_t, range, range)
)

TEST(pcl, Cleanup)
{
  EXPECT_NO_THROW(fs::remove(fs::path(H5_INFILE)));
}

TEST(pcl, PointXYZ)
{
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");
  std::vector<int> rows_list{16, 32, 64, 128};
  int cols = 2048;

  for (int rows : rows_list)
    {
      int npts = rows * cols;
      auto vec_x = h5_bridge::random_vec<float>(npts);
      auto vec_y = h5_bridge::random_vec<float>(npts);
      auto vec_z = h5_bridge::random_vec<float>(npts);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
      cloud->height = rows;
      cloud->width = cols;
      cloud->points.resize(cloud->width * cloud->height);

      for (std::size_t i = 0; i < npts; ++i)
        {
          cloud->points[i].x = vec_x[i];
          cloud->points[i].y = vec_y[i];
          cloud->points[i].z = vec_z[i];
        }

      sensor_msgs::msg::PointCloud2 msg;
      EXPECT_NO_THROW(pcl::toROSMsg(*cloud, msg));

      EXPECT_NO_THROW(
        h5b_sensor_msgs::write(
          h5.get(), "/pcl/xyz/" + std::to_string(rows), msg));
    }
}

TEST(pcl, PointXYZI)
{
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");
  std::vector<int> rows_list{16, 32, 64, 128};
  int cols = 2048;

  for (int rows : rows_list)
    {
      int npts = rows * cols;
      auto vec_x = h5_bridge::random_vec<float>(npts);
      auto vec_y = h5_bridge::random_vec<float>(npts);
      auto vec_z = h5_bridge::random_vec<float>(npts);
      auto vec_i = h5_bridge::random_vec<float>(npts);

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
      cloud->height = rows;
      cloud->width = cols;
      cloud->points.resize(cloud->width * cloud->height);

      for (std::size_t i = 0; i < npts; ++i)
        {
          cloud->points[i].x = vec_x[i];
          cloud->points[i].y = vec_y[i];
          cloud->points[i].z = vec_z[i];
          cloud->points[i].intensity = vec_i[i];
        }

      sensor_msgs::msg::PointCloud2 msg;
      EXPECT_NO_THROW(pcl::toROSMsg(*cloud, msg));

      EXPECT_NO_THROW(
        h5b_sensor_msgs::write(
          h5.get(), "/pcl/xyzi/" + std::to_string(rows), msg));
    }
}

TEST(pcl, PointOuster)
{
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");
  std::vector<int> rows_list{16, 32, 64, 128};
  int cols = 2048;

  for (int rows : rows_list)
    {
      int npts = rows * cols;

      auto vec_x = h5_bridge::random_vec<float>(npts);
      auto vec_y = h5_bridge::random_vec<float>(npts);
      auto vec_z = h5_bridge::random_vec<float>(npts);
      auto vec_i = h5_bridge::random_vec<float>(npts);
      auto vec_t = h5_bridge::random_vec<std::uint32_t>(npts);
      auto vec_reflectivity = h5_bridge::random_vec<std::uint16_t>(npts);
      auto vec_ring_ = h5_bridge::random_vec<std::uint8_t>(npts);
      auto vec_noise = h5_bridge::random_vec<std::uint16_t>(npts);
      auto vec_range = h5_bridge::random_vec<std::uint32_t>(npts);

      std::vector<std::uint8_t> vec_ring;
      std::transform(vec_ring_.begin(), vec_ring_.end(),
                     std::back_inserter(vec_ring),
                     [rows](std::uint8_t r) -> std::uint8_t
                     { return r % rows; });

      pcl::PointCloud<PointOS>::Ptr cloud(new pcl::PointCloud<PointOS>);
      cloud->height = rows;
      cloud->width = cols;
      cloud->points.resize(npts);

      for (std::size_t i = 0; i < npts; ++i)
        {
          cloud->points[i].x = vec_x[i];
          cloud->points[i].y = vec_y[i];
          cloud->points[i].z = vec_z[i];
          cloud->points[i].intensity = vec_i[i];
          cloud->points[i].t = vec_t[i];
          cloud->points[i].reflectivity = vec_reflectivity[i];
          cloud->points[i].ring = vec_ring[i];
          cloud->points[i].noise = vec_noise[i];
          cloud->points[i].range = vec_range[i];
        }

      sensor_msgs::msg::PointCloud2 msg;
      EXPECT_NO_THROW(pcl::toROSMsg(*cloud, msg));

      EXPECT_NO_THROW(
        h5b_sensor_msgs::write(
          h5.get(), "/pcl/ouster/" + std::to_string(rows), msg));
    }
}
