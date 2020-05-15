// -*- c++ -*-
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
#ifndef H5B_OPENCV__H5B_OPENCV_H_
#define H5B_OPENCV__H5B_OPENCV_H_

#include <string>
#include <opencv2/core.hpp>
#include <h5_bridge/h5_file.hpp>

namespace h5b_opencv
{
  /**
   * Writes and OpenCV array to an HDF5 dataset
   *
   * @param[in] h5 A pointer to an open hdf5 file
   * @param[in] dset The full path to the dataset to write
   * @param[in] mat The OpenCV array to write to h5
   * @param[in] gzip The gzip compression level
   */
  void write(h5_bridge::H5File * h5, const std::string& dset,
             const cv::Mat& mat, int gzip = 0);

  /**
   * @param[in] h5 A pointer to an open hdf5 file
   * @param[in] dset The full path to the dataset to read from
   *
   * @return A cv::Mat encoding of the hdf5 dataset.
   */
  cv::Mat read(h5_bridge::H5File * h5, const std::string& dset);

} // end: namespace h5b_opencv

#endif // H5B_OPENCV__H5B_OPENCV_H_
