/*
 * Copyright (C) 2020 Box Robotics, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <h5b_opencv.hpp>
#include <cstdint>
#include <string>
#include <opencv2/core.hpp>
#include <h5_bridge/h5_file.hpp>
#include <h5_bridge/h5_dset_t.hpp>
#include <h5_bridge/err.hpp>

struct H5DTypeEncodingVisitor
{
  int operator() (std::uint8_t) const { return CV_8U; }
  int operator() (std::int8_t) const { return CV_8S; }
  int operator() (std::uint16_t) const { return CV_16U; }
  int operator() (std::int16_t) const { return CV_16S; }
  int operator() (std::uint32_t) const { return -1; }
  int operator() (std::int32_t) const { return CV_32S; }
  int operator() (float) const { return CV_32F; }
  int operator() (double) const { return CV_64F; }
};

void
h5b_opencv::write(h5_bridge::H5File * h5, const std::string& dset,
                  const cv::Mat& mat, int gzip)
{
  h5_bridge::DSet_t dtype;

  std::uint8_t depth = mat.type() & CV_MAT_DEPTH_MASK;
  switch (depth)
    {
    case CV_8U:  dtype = std::uint8_t{0};  break;
    case CV_8S:  dtype = std::int8_t{0};   break;
    case CV_16U: dtype = std::uint16_t{0}; break;
    case CV_16S: dtype = std::int16_t{0};  break;
    case CV_32S: dtype = std::int32_t{0};  break;
    case CV_32F: dtype = float{0};         break;
    case CV_64F: dtype = double{0};        break;

    default:
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_INVALID_TYPE));
    }

  h5->write(dset, mat.ptr(), dtype, mat.rows, mat.cols, mat.channels(), gzip);
}

cv::Mat
h5b_opencv::read(h5_bridge::H5File * h5, const std::string& dset)
{
  cv::Mat mat;
  auto [tp, rows, cols, chans] = h5->get_shape(dset);
  int encoding = std::visit(H5DTypeEncodingVisitor(), tp);
  if (encoding < 0)
    {
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_INVALID_TYPE));
    }

  mat.create(rows, cols, CV_MAKETYPE(encoding, chans));
  h5->read(dset, mat.ptr());

  return mat;
}
