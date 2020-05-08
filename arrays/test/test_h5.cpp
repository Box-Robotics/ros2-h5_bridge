#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <arrays/err.hpp>
#include <arrays/h5_bridge.hpp>
#include <arrays/logging.hpp>

#include <gtest/gtest.h>

namespace fs = std::filesystem;

auto h5_infile_ = []()->std::string
{
  auto tmp_dir = std::filesystem::temp_directory_path();
  return tmp_dir.native() + std::string("/arrays_test.h5");
};

const std::string H5_INFILE = h5_infile_();
// XXX: Don't change these, the tests below depnd on them.
const std::vector<std::string> GROUP_NAMES =
  {
    "/arrays/A", "/arrays/B", "/arrays/C",
    "/arrays/A/1", "/arrays/A/2", "/arrays/A/3",
    "/arrays/B/4", "/arrays/B/5", "/arrays/B/6",
    "/arrays/C/7", "/arrays/C/8", "/arrays/C/9",
  };

TEST(h5, FileCreationBadMode)
{
  fs::remove(fs::path(H5_INFILE));
  arrays::use_arrays_logger();
  ALOG_INFO("==========================================================");

  bool ex_caught = false;

  try
    {
      auto h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "foo");
    }
  catch (const arrays::error_t& ex)
    {
      ex_caught = true;
      EXPECT_EQ(ex.code(), arrays::ERR_H5_BAD_MODE);
    }

  EXPECT_TRUE(ex_caught);
}

TEST(h5, FileCreation)
{
  fs::remove(fs::path(H5_INFILE));

  arrays::h5_bridge::H5File::Ptr h5;

  EXPECT_THROW(h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "r"),
               arrays::error_t);

  EXPECT_THROW(
    h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "r+"),
    arrays::error_t);

  EXPECT_NO_THROW(
    h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "w"));

  EXPECT_THROW(
    h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "w-"),
    arrays::error_t);

  EXPECT_NO_THROW(
    h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "a"));
  h5.reset();

  fs::remove(fs::path(H5_INFILE));

  EXPECT_NO_THROW(
    h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "a"));
}

TEST(h5, MoveSemantics)
{
  auto h5 = arrays::h5_bridge::H5File(H5_INFILE, "a");
  auto h5_new = arrays::h5_bridge::H5File(std::move(h5));
  EXPECT_STREQ(h5_new.filename().c_str(), H5_INFILE.c_str());
}

TEST(h5, filename)
{
  auto h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "a");
  EXPECT_STREQ(h5->filename().c_str(), H5_INFILE.c_str());
}

TEST(h5, group)
{
  auto h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "a");

  for (auto& name : GROUP_NAMES)
    {
      arrays::h5_bridge::H5ObjId grp;
      EXPECT_NO_THROW(grp = h5->group(name));
      EXPECT_TRUE(grp);
      EXPECT_STREQ(grp.value().c_str(), name.c_str());
    }
}

TEST(h5, subgroups)
{
  auto h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "a");

  arrays::h5_bridge::H5ObjId grp; // std::nullopt -> get subgroups of "/"
  auto subgroups = h5->subgroups(grp);
  EXPECT_TRUE(subgroups == decltype(subgroups)({"arrays"}));

  subgroups = h5->subgroups("/arrays");
  EXPECT_TRUE(subgroups == decltype(subgroups)({"A", "B", "C"}));

  subgroups = h5->subgroups("/arrays/A");
  EXPECT_TRUE(subgroups == decltype(subgroups)({"1", "2", "3"}));

  subgroups = h5->subgroups("/arrays/B");
  EXPECT_TRUE(subgroups == decltype(subgroups)({"4", "5", "6"}));

  subgroups = h5->subgroups("/arrays/C");
  EXPECT_TRUE(subgroups == decltype(subgroups)({"7", "8", "9"}));

  EXPECT_THROW(h5->subgroups("/foo/bar/baz"), arrays::error_t);
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

  auto h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "a");
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

TEST(h5, flush)
{
  auto h5 = std::make_unique<arrays::h5_bridge::H5File>(H5_INFILE, "a");
  EXPECT_NO_THROW(h5->flush());
}
