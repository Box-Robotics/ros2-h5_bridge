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

void
h5b::write(h5_bridge::H5File * h5, const std::string& dset,
           const sensor_msgs::msg::Image& img, int gzip)
{
  if (img.width < 1)
    {
      throw(std::domain_error("Image cannot have zero width!"));
    }

  //
  // XXX: This big if/else chain needs to be much more elegant.
  //

  if (img.encoding.rfind("8U", 0))
    {
      h5->write<std::uint8_t>(
        dset,
        img.data.data(),
        static_cast<int>(img.height),
        static_cast<int>(img.width),
        static_cast<int>(img.step) / static_cast<int>(img.width),
        gzip);
    }
  else if (img.encoding.rfind("8S", 0))
    {
      h5->write<std::int8_t>(
        dset,
        img.data.data(),
        static_cast<int>(img.height),
        static_cast<int>(img.width),
        static_cast<int>(img.step) / static_cast<int>(img.width),
        gzip);
    }
  else if (img.encoding.rfind("16U", 0))
    {
      h5->write<std::uint16_t>(
        dset,
        img.data.data(),
        static_cast<int>(img.height),
        static_cast<int>(img.width),
        static_cast<int>(img.step) / static_cast<int>(img.width),
        gzip);
    }
  else if (img.encoding.rfind("16S", 0))
    {
      h5->write<std::int16_t>(
        dset,
        img.data.data(),
        static_cast<int>(img.height),
        static_cast<int>(img.width),
        static_cast<int>(img.step) / static_cast<int>(img.width),
        gzip);
    }
  else if (img.encoding.rfind("32S", 0))
    {
      h5->write<std::int32_t>(
        dset,
        img.data.data(),
        static_cast<int>(img.height),
        static_cast<int>(img.width),
        static_cast<int>(img.step) / static_cast<int>(img.width),
        gzip);
    }
  else if (img.encoding.rfind("32F", 0))
    {
      h5->write<float>(
        dset,
        img.data.data(),
        static_cast<int>(img.height),
        static_cast<int>(img.width),
        static_cast<int>(img.step) / static_cast<int>(img.width),
        gzip);
    }
  else if (img.encoding.rfind("64F", 0))
    {
      h5->write<double>(
        dset,
        img.data.data(),
        static_cast<int>(img.height),
        static_cast<int>(img.width),
        static_cast<int>(img.step) / static_cast<int>(img.width),
        gzip);
    }
  else
    {
      throw(std::domain_error("Unsupported encoding: " + img.encoding));
    }

  h5->set_attr(dset, "header.frame_id", img.header.frame_id);
  h5->set_attr(dset, "header.stamp.sec", img.header.stamp.sec);
  h5->set_attr(dset, "header.stamp.nanosec", img.header.stamp.nanosec);
  h5->set_attr(dset, "height", img.height);
  h5->set_attr(dset, "width" , img.width);
  h5->set_attr(dset, "encoding", std::string(img.encoding));
  h5->set_attr(dset, "is_bigendian", img.is_bigendian);
  h5->set_attr(dset, "step", img.step);
}

sensor_msgs::msg::Image
h5b::toImageMsg(h5_bridge::H5File * h5, const std::string& dset)
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

  im.encoding =
    std::visit([chans] (const auto& val) -> std::string
               {
                 std::string prefix;
                 if (std::is_same_v<decltype(val), std::uint8_t>)
                   {
                     prefix = "8UC";
                   }
                 else if (std::is_same_v<decltype(val), std::int8_t>)
                   {
                     prefix = "8SC";
                   }
                 else if (std::is_same_v<decltype(val), std::uint16_t>)
                   {
                     prefix = "16UC";
                   }
                 else if (std::is_same_v<decltype(val), std::int16_t>)
                   {
                     prefix = "16SC";
                   }
                 else if (std::is_same_v<decltype(val), std::int32_t>)
                   {
                     prefix = "32SC";
                   }
                 else if (std::is_same_v<decltype(val), float>)
                   {
                     prefix = "32FC";
                   }
                 else if (std::is_same_v<decltype(val), double>)
                   {
                     prefix = "64FC";
                   }
                 else
                   {
                     return "unknown";
                   }

                 switch (chans)
                   {
                   case 1:
                     return prefix + "1";
                   case 2:
                     return prefix + "2";
                   case 3:
                     return prefix + "3";
                   case 4:
                     return prefix + "4";
                   default:
                     break;
                   }

                 return "unknown";
               }, tp);
  if (im.encoding == "unknown")
    {
      try
        {
          im.encoding = h5->attr<std::string>(dset, "encoding");
        }
      catch (const h5_bridge::error_t& ex)
        {
          // noop
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
