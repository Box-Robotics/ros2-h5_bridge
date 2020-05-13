#include <h5b_sensor_msgs/image.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <variant>
#include <h5_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace enc = sensor_msgs::image_encodings;

struct H5DTypeEncodingVisitor
{
  std::string operator() (std::uint8_t) const { return "8U"; }
  std::string operator() (std::int8_t) const { return "8S"; }
  std::string operator() (std::uint16_t) const { return "16U"; }
  std::string operator() (std::int16_t) const { return "16S"; }
  std::string operator() (std::int32_t) const { return "32S"; }
  std::string operator() (float) const { return "32F"; }
  std::string operator() (double) const { return "64F"; }
};

template<typename T>
void
write_wrapper(h5_bridge::H5File * h5, const std::string& dset,
              const sensor_msgs::msg::Image& img, int gzip)
{
  h5->write<T>(
    dset, img.data.data(),
    static_cast<int>(img.height),
    static_cast<int>(img.width),
    static_cast<int>(img.step) / (static_cast<int>(img.width) * sizeof(T)),
    gzip);
}

void
h5b_sensor_msgs::write(h5_bridge::H5File * h5, const std::string& dset,
                       const sensor_msgs::msg::Image& img, int gzip)
{
  if (img.width < 1)
    {
      throw(std::domain_error("Image cannot have zero width!"));
    }

  if (img.encoding.rfind("8U", 0) == 0)
    {
      write_wrapper<std::uint8_t>(h5, dset, img, gzip);
    }
  else if (img.encoding.rfind("8S", 0) == 0)
    {
      write_wrapper<std::int8_t>(h5, dset, img, gzip);
    }
  else if (img.encoding.rfind("16U", 0) == 0)
    {
      write_wrapper<std::uint16_t>(h5, dset, img, gzip);
    }
  else if (img.encoding.rfind("16S", 0) == 0)
    {
      write_wrapper<std::int16_t>(h5, dset, img, gzip);
    }
  else if (img.encoding.rfind("32S", 0) == 0)
    {
      write_wrapper<std::int32_t>(h5, dset, img, gzip);
    }
  else if (img.encoding.rfind("32F", 0) == 0)
    {
      write_wrapper<float>(h5, dset, img, gzip);
    }
  else if (img.encoding.rfind("64F", 0) == 0)
    {
      write_wrapper<double>(h5, dset, img, gzip);
    }
  else
    {
      H5B_ERROR("Could not write {} with encoding '{}'",
                dset, img.encoding);
      throw(std::domain_error("Unsupported encoding: " + img.encoding));
    }

  h5->set_attr(dset, "header.frame_id", img.header.frame_id);
  h5->set_attr(dset, "header.stamp.sec", img.header.stamp.sec);
  h5->set_attr(dset, "header.stamp.nanosec", img.header.stamp.nanosec);
  h5->set_attr(dset, "encoding", std::string(img.encoding));
  h5->set_attr(dset, "is_bigendian", img.is_bigendian);
}

sensor_msgs::msg::Image
h5b_sensor_msgs::toImageMsg(h5_bridge::H5File * h5, const std::string& dset)
{
  sensor_msgs::msg::Image im;
  auto [tp, rows, cols, chans] = h5->get_shape(dset);
  im.height = rows;
  im.width = cols;
  auto pixel_size = std::visit([] (const auto& val) -> std::size_t
                               { return sizeof(decltype(val)); }, tp);
  im.step = cols * chans * pixel_size;
  im.data.resize(rows * im.step);
  h5->read(dset, reinterpret_cast<std::uint8_t *>(im.data.data()));

  im.encoding = std::visit(H5DTypeEncodingVisitor(), tp);
  switch (chans)
    {
    case 1:
      im.encoding += "C1";
      break;

    case 2:
      im.encoding += "C2";
      break;

    case 3:
      im.encoding += "C3";
      break;

    case 4:
      im.encoding += "C4";
      break;

    default:
      im.encoding = "unknown";
      break;
    }

  if (im.encoding == "unknown")
    {
      try
        {
          auto d = h5->dset(dset);
          im.encoding = h5->attr<std::string>(d, "encoding");
        }
      catch (const h5_bridge::error_t& ex)
        {
          H5B_WARN("Could not pull encoding as attribute on data set: {}",
                   dset);
          H5B_WARN("{}", ex.what());
        }
    }

  try
    {
      im.is_bigendian = h5->attr<std::uint8_t>(dset, "is_bigendian");
    }
  catch (const h5_bridge::error_t& ex)
    {
      im.is_bigendian = h5_bridge::big_endian() ? 0x1 : 0x0;
    }

  try
    {
      im.header.frame_id = h5->attr<std::string>(dset, "header.frame_id");
    }
  catch (const h5_bridge::error_t& ex)
    {
      im.header.frame_id = "";
    }

  try
    {
      im.header.stamp.sec =
        h5->attr<std::int32_t>(dset, "header.stamp.sec");
      im.header.stamp.nanosec =
        h5->attr<std::uint32_t>(dset, "header.stamp.nanosec");
    }
  catch (const h5_bridge::error_t& ex)
    {
      im.header.stamp.sec = 0;
      im.header.stamp.nanosec = 0;
    }

  return im;
}
