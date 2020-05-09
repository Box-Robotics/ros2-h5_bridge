#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <h5_bridge/err.hpp>
#include <h5_bridge/h5_file.hpp>
#include <h5_bridge/logging.hpp>
#include <h5_bridge/util.hpp>

#include <gtest/gtest.h>

namespace fs = std::filesystem;

auto h5_infile_ = []()->std::string
{
  auto tmp_dir = std::filesystem::temp_directory_path();
  return tmp_dir.native() + std::string("/h5_bridge_test.h5");
};

const std::string H5_INFILE = h5_infile_();
// XXX: Don't change these, the tests below depnd on them.
const std::vector<std::string> GROUP_NAMES =
  {
    "/h5_bridge/A", "/h5_bridge/B", "/h5_bridge/C",
    "/h5_bridge/A/1", "/h5_bridge/A/2", "/h5_bridge/A/3",
    "/h5_bridge/B/4", "/h5_bridge/B/5", "/h5_bridge/B/6",
    "/h5_bridge/C/7", "/h5_bridge/C/8", "/h5_bridge/C/9",
  };

TEST(h5, FileCreationBadMode)
{
  fs::remove(fs::path(H5_INFILE));
  h5_bridge::use_h5_bridge_logger();
  H5B_INFO("==========================================================");

  bool ex_caught = false;

  try
    {
      auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "foo");
    }
  catch (const h5_bridge::error_t& ex)
    {
      ex_caught = true;
      EXPECT_EQ(ex.code(), h5_bridge::ERR_H5_BAD_MODE);
    }

  EXPECT_TRUE(ex_caught);
}

TEST(h5, FileCreation)
{
  fs::remove(fs::path(H5_INFILE));

  h5_bridge::H5File::Ptr h5;

  EXPECT_THROW(h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "r"),
               h5_bridge::error_t);

  EXPECT_THROW(
    h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "r+"),
    h5_bridge::error_t);

  EXPECT_NO_THROW(
    h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "w"));

  EXPECT_THROW(
    h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "w-"),
    h5_bridge::error_t);

  EXPECT_NO_THROW(
    h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a"));
  h5.reset();

  fs::remove(fs::path(H5_INFILE));

  EXPECT_NO_THROW(
    h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a"));
}

TEST(h5, MoveSemantics)
{
  auto h5 = h5_bridge::H5File(H5_INFILE, "a");
  auto h5_new = h5_bridge::H5File(std::move(h5));
  EXPECT_STREQ(h5_new.filename().c_str(), H5_INFILE.c_str());
}

TEST(h5, filename)
{
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");
  EXPECT_STREQ(h5->filename().c_str(), H5_INFILE.c_str());
}

TEST(h5, group)
{
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");

  for (auto& name : GROUP_NAMES)
    {
      h5_bridge::H5ObjId grp;
      EXPECT_NO_THROW(grp = h5->group(name));
      EXPECT_TRUE(grp);
      EXPECT_STREQ(grp.value().c_str(), name.c_str());
    }
}

TEST(h5, subgroups)
{
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");

  h5_bridge::H5ObjId grp; // std::nullopt -> get subgroups of "/"
  auto subgroups = h5->subgroups(grp);
  EXPECT_TRUE(subgroups == decltype(subgroups)({"h5_bridge"}));

  subgroups = h5->subgroups("/h5_bridge");
  EXPECT_TRUE(subgroups == decltype(subgroups)({"A", "B", "C"}));

  subgroups = h5->subgroups("/h5_bridge/A");
  EXPECT_TRUE(subgroups == decltype(subgroups)({"1", "2", "3"}));

  subgroups = h5->subgroups("/h5_bridge/B");
  EXPECT_TRUE(subgroups == decltype(subgroups)({"4", "5", "6"}));

  subgroups = h5->subgroups("/h5_bridge/C");
  EXPECT_TRUE(subgroups == decltype(subgroups)({"7", "8", "9"}));

  EXPECT_THROW(h5->subgroups("/foo/bar/baz"), h5_bridge::error_t);
}

TEST(h5, GroupAttributes)
{
  std::uint8_t u8 = 8;
  std::uint16_t u16 = 16;
  std::uint32_t u32 = 32;
  std::uint64_t u64 = 64;
  std::int8_t s8 = -8;
  std::int16_t s16 = -16;
  std::int32_t s32 = -32;
  std::int64_t s64 = -64;
  float f32 = 32.0;
  double f64 = 64.0;
  bool bt = true;
  bool bf = false;
  std::string foo("foo");
  std::string bar("bar");

  std::vector<std::string> attr_names =
    {"u8", "u16", "u32", "u64", "s8", "s16", "s32", "s64",
     "f32", "f64", "bt", "bf", "foo", "bar"};
  std::sort(attr_names.begin(), attr_names.end());

  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");
  for (auto& name : GROUP_NAMES)
    {
      auto grp = h5->group(name);

      EXPECT_NO_THROW(h5->set_attr(grp, "u8", u8));
      EXPECT_NO_THROW(h5->set_attr(grp, "u16", u16));
      EXPECT_NO_THROW(h5->set_attr(grp, "u32", u32));
      EXPECT_NO_THROW(h5->set_attr(grp, "u64", u64));
      EXPECT_NO_THROW(h5->set_attr(grp, "s8", s8));
      EXPECT_NO_THROW(h5->set_attr(grp, "s16", s16));
      EXPECT_NO_THROW(h5->set_attr(grp, "s32", s32));
      EXPECT_NO_THROW(h5->set_attr(grp, "s64", s64));
      EXPECT_NO_THROW(h5->set_attr(grp, "f32", f32));
      EXPECT_NO_THROW(h5->set_attr(grp, "f64", f64));
      EXPECT_NO_THROW(h5->set_attr(grp, "bt", bt));
      EXPECT_NO_THROW(h5->set_attr(grp, "bf", bf));
      EXPECT_NO_THROW(h5->set_attr(grp, "foo", foo));
      EXPECT_NO_THROW(h5->set_attr(grp, "bar", bar.c_str()));

      auto attrs = h5->attributes(grp);
      std::sort(attrs.begin(), attrs.end());
      EXPECT_TRUE(attr_names == attrs);

      EXPECT_EQ(u8, h5->attr<std::uint8_t>(grp, "u8"));
      EXPECT_EQ(u16, h5->attr<std::uint16_t>(grp, "u16"));
      EXPECT_EQ(u32, h5->attr<std::uint32_t>(grp, "u32"));
      EXPECT_EQ(u64, h5->attr<std::uint64_t>(grp, "u64"));
      EXPECT_EQ(s8, h5->attr<std::int8_t>(grp, "s8"));
      EXPECT_EQ(s16, h5->attr<std::int16_t>(grp, "s16"));
      EXPECT_EQ(s32, h5->attr<std::int32_t>(grp, "s32"));
      EXPECT_EQ(s64, h5->attr<std::int64_t>(grp, "s64"));
      EXPECT_TRUE(h5->attr<bool>(grp, "bt"));
      EXPECT_FALSE(h5->attr<bool>(grp, "bf"));
      EXPECT_FLOAT_EQ(f32, h5->attr<float>(grp, "f32"));
      EXPECT_FLOAT_EQ(f64, h5->attr<double>(grp, "f64"));
      EXPECT_STREQ(foo.c_str(), h5->attr<std::string>(grp, "foo").c_str());
      EXPECT_STREQ(bar.c_str(), h5->attr<std::string>(grp, "bar").c_str());
    }
}

TEST(h5, DSetRW)
{
  auto rw = [](auto Tp, const std::string& path,
               int rows, int cols, int chans)
    {
      auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");
      h5_bridge::H5ObjId dset;

      auto vec = h5_bridge::random_vec<decltype(Tp)>(rows*cols*chans);
      EXPECT_NO_THROW(h5->write(path, vec, rows, cols, chans));
      dset = h5->dset(path);
      EXPECT_FALSE(dset == std::nullopt);
      EXPECT_NO_THROW(h5->set_attr(dset, "rows", rows));
      EXPECT_NO_THROW(h5->set_attr(dset, "cols", cols));
      EXPECT_NO_THROW(h5->set_attr(dset, "chans", chans));

      //
      // XXX: read back the data and make sure it looks good
      //
    };

  //
  // Vector
  //
  int rows = 100;
  int cols = 1;
  int chans = 1;

  rw(std::uint8_t{0}, "/vector/u8", rows, cols, chans);
  rw(std::uint16_t{0}, "/vector/u16", rows, cols, chans);
  rw(std::int8_t{0}, "/vector/s8", rows, cols, chans);
  rw(std::int16_t{0}, "/vector/s16", rows, cols, chans);
  rw(std::int32_t{0}, "/vector/s32", rows, cols, chans);
  rw(float{0.}, "/vector/f32", rows, cols, chans);
  rw(double{0.}, "/vector/f64", rows, cols, chans);

  //
  // VGA grayscale image
  //
  rows = 480;
  cols = 640;
  chans = 1;

  rw(std::uint8_t{0}, "/image/mono/u8", rows, cols, chans);
  rw(std::uint16_t{0}, "/image/mono/u16", rows, cols, chans);
  rw(std::int8_t{0}, "/image/mono/s8", rows, cols, chans);
  rw(std::int16_t{0}, "/image/mono/s16", rows, cols, chans);
  rw(std::int32_t{0}, "/image/mono/s32", rows, cols, chans);
  rw(float{0.}, "/image/mono/f32", rows, cols, chans);
  rw(double{0.}, "/image/mono/f64", rows, cols, chans);

  //
  // VGA rgb image
  //
  rows = 480;
  cols = 640;
  chans = 3;

  rw(std::uint8_t{0}, "/image/rgb/u8", rows, cols, chans);
  rw(std::uint16_t{0}, "/image/rgb/u16", rows, cols, chans);
  rw(std::int8_t{0}, "/image/rgb/s8", rows, cols, chans);
  rw(std::int16_t{0}, "/image/rgb/s16", rows, cols, chans);
  rw(std::int32_t{0}, "/image/rgb/s32", rows, cols, chans);
  rw(float{0.}, "/image/rgb/f32", rows, cols, chans);
  rw(double{0.}, "/image/rgb/f64", rows, cols, chans);

  //
  // LiDAR-like "images" XYZI
  //
  rows = 16;
  cols = 2048;
  chans = 4;

  rw(std::uint8_t{0}, "/lidar/16/u8", rows, cols, chans);
  rw(std::uint16_t{0}, "/lidar/16/u16", rows, cols, chans);
  rw(std::int8_t{0}, "/lidar/16/s8", rows, cols, chans);
  rw(std::int16_t{0}, "/lidar/16/s16", rows, cols, chans);
  rw(std::int32_t{0}, "/lidar/16/s32", rows, cols, chans);
  rw(float{0.}, "/lidar/16/f32", rows, cols, chans);
  rw(double{0.}, "/lidar/16/f64", rows, cols, chans);
}



TEST(h5, flush)
{
  auto h5 = std::make_unique<h5_bridge::H5File>(H5_INFILE, "a");
  EXPECT_NO_THROW(h5->flush());
}
