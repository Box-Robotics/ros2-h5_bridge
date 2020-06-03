ros2-h5_bridge
==============
HDF5 for humans. This library provides an intuitive C++ interface for
HDF5 serialization and deserialization of various array data types used in
ROS2.

Releases
========
<table>
  <tr>
    <th>ros2-h5_bridge version</th>
    <th>Linux distribution</th>
    <th>ROS distribution</th>
  </tr>
  <tr>
    <td>0.1.0</td>
    <td>Ubuntu 18.04</td>
    <td>Eloquent</td>
  </tr>
</table>

About
=====
From the [HDF Group
Website](https://support.hdfgroup.org/HDF5/whatishdf5.html), HDF5 is a unique
technology suite that makes possible the management of extremely large and
complex data collections. This collection of packages attempts to make
interfacing to HDF5 seamless for usage within ROS2 software systems.

The design of `h5_bridge` is intended to be simple for end-users. Under the
hood, the HDF5 bits are completely hidden from the programmer to include
compiling and linking (the core interface is implemented as a PIMPL). It is
advisable to make your primary _handle_ to the h5 file a move-only unique_ptr
type (see the code examples below). While the library provides no explicit
thread safety, a simple lock around the `h5_file` in application code will
ensure the library can be used from multiple threads.

`h5_bridge` never returns a native HDF5 type back to the user. Rather, it holds
a private cache of used HDF5 objects (primarily groups and data sets) and gives
the user a handle or pointer (a simple string "key") to the object held in the
underlying cache. The benefit of this approach is that we can apply the PIMPL
pattern to keep usage of the library simple. The down side is that, depending
upon the applicaiton, the user may need to do some cache management. The cache
will not explicitly clear until the `h5_file` pointer goes out of
scope. However, the `clear(...)` function provided by the interface allows
for explicit cache management.

Another important design decision of `h5_bridge` is that data are serialized in
a way to support analysis. Images are serialized with proper shape and with the
pixel data type written to h5 explicitly. We are not just writing bytes to H5
with some metadata annotating what the bytes mean. If you are looking for an
I/O optimized H5 interface, this is not the library for you. If, however, you
are looking for a tool to assist in robotics/ROS2-based data science, this
library should treat you quite well. Indeed, serializing ROS2 array types with
this library allows for intuitive analysis of your data using standard H5
tools. A favorite of ours is
[HDFView](https://www.hdfgroup.org/downloads/hdfview/).


Building, Testing, and Installing
=================================
The build command we use is:

```
$ colcon build \
  --cmake-args " -DCMAKE_BUILD_TYPE=Release" \
  --event-handlers console_cohesion+
```

To run the unit tests (recommended), we like to see the `gtest` output in
real-time on the screen. To that end, we don't use the `colcon` approach for
this step. Instead, we use the following `test.sh` script, run from the
top-level of our workspace.

```
#!/bin/bash

echo -e "\nh5_bridge:\n"
H5_BRIDGE_LOG_FILE=/tmp/h5_bridge.log ./build/h5_bridge/h5_bridge_test
echo -e "\nh5b_sensor_msgs:\n"
H5_BRIDGE_LOG_FILE=/tmp/h5_bridge.log ./build/h5b_sensor_msgs/h5b_sensor_msgs_test
echo -e "\nh5b_opencv:\n"
H5_BRIDGE_LOG_FILE=/tmp/h5_bridge.log ./build/h5b_opencv/h5b_opencv_test
```

To install the software to a location other than the `install` subdirectory of
your workspace, you can run the `colcon build ...` command again with the
`--install-base` argument. For example:

```
#!/bin/bash

rm -rf build log install

colcon build \
  --install-base "${BOX_ROS2}/h5_bridge" \
  --cmake-args " -DCMAKE_BUILD_TYPE=Release" \
  --event-handlers console_cohesion+
```

In the example above, we have an environment variable called `${BOX_ROS2}` set
that points to a top-level directory of where we install custom ROS2
packages. Adapt the above as you see fit for your system.


Supported Types
===============
The core package in this project is the `h5_bridge`. It supports HDF5 file
manipulation in a very generic way. This includes creating/opening files,
creating groups and datasets, writing attributes (annotating) H5 entitites,
etc. Serialization and deserialization of data types is handled by subpackages
that use the facilities of `h5_bridge`. Below is a list of packages and the
data types they support for marshaling data to/from HDF5.


h5_bridge
---------
**NOTE:** More comprehensive example code can be found in the [unit
tests](h5_bridge/test/test_h5.cpp).


### std::vector<T>

The core `h5_bridge` package provides a simple way to write arrays of
`std::vector<T>` to HDF5. This provides a foundation that all other read/write
implementations can leverage. Writing a std vector to HDF5 is as simple as:

```cpp
// open the file
auto h5 = std::make_unique<h5_bridge::H5File>("/path/to/file.h5", "a");

// create a vector of random pixel data
auto vec = h5_bridge::random_vec<T>(rows*cols*chans);

// write to a data set
h5->write("/path/to/dataset", vec, rows, cols, chans);
```

Reading looks like:

```cpp
auto [vec, rows, cols, chans] = h5->read<T>("/path/to/dataset");
```

h5b_sensor_msgs
--------------
### sensor_msgs::msg::Image
**NOTE:** More comprehensive example code can be found in the [unit
tests](h5b_sensor_msgs/test/test_image.cpp).


Reading and writing an image looks like:

```cpp
//
// Here, `h5` is the `H5File` pointer (see above example) and `path` is a
// `std::string` encoding the full path to the data set to read in as an
// image.
//
auto im = h5b_sensor_msgs::read<sensor_msgs::msg::Image>(h5.get(), path);

// writing the data back out...
h5b_sensor_msgs::write(h5.get(), "/path/to/new/image", im);
```

When writing an image to h5, the ROS2 message header information gets annotated
to the dataset as a set of attributes. The data are written as an organized
matrix of pixels according to its shape as specified in the message metadata.

### sensor_msgs::msg::PointCloud2
**NOTE:** More comprehensive example code can be found in the [unit
tests](h5b_sensor_msgs/test/test_pcl.cpp).


When developing the serialization of the `PointCloud2` type we took an
opinionated approach. As they are encoded in the ROS2 message type,
`PointCloud2` follows the _array of structs_ approach. That is, each point
(struct) is packed together and serialized linearly together in memory. In our
approach, we employ the _struct of arrays_ type of serialization. That is, we
break apart each field of the point struct and serialize each as an array (h5
data set). To that end, the path to a point cloud in the H5 file is actually a
group and not a dataset. The datasets that sit beneath that group, represent
the data arrays. The one minor exception to this rule is that if the
`PointCloud` has `x`, `y`, and `z` fields, we pack those together and serialize
them as a single 3-channel dataset. To do this, we assume all three fields have
the same data type and that from the developer's perspective, you will likely
want to operate on those Cartesian data together in a loop so this is a more
cache efficient data structure.

Here is some example code on using `PointCloud` data in this library:

```cpp
// read
auto msg =
  h5b_sensor_msgs::read<sensor_msgs::msg::PointCloud2>(
    h5.get(), "/path/to/pcl_group");

// write
h5b_sensor_msgs::write(h5.get(), "/path/to/pcl_group", msg);
```

h5b_opencv
----------
### cv::Mat
**NOTE:** More comprehensive example code can be found in the [unit
tests](h5b_opencv/test/test_opencv.cpp).


OpenCV messages are serialized in a similar way as the
`sensor_msgs::msg::Image` is handled. That is, an organized array of pixel
data. The interface into the library is via the _type-erased_ versions of
`cv::Mat`. However, since `cv::Mat<T>` is a subclass of `cv::Mat` the strongly
typed versions can be written directly using this library. To be clear, what
will get returned in a call from `read` is a type-erased `cv::Mat`, but, that
can be converted to `cv::Mat<T>` quite easily in application code.

```cpp

cv::Mat mat = ...

// write
h5b_opencv::write(h5.get(), "/path/to/mat", mat);

// read
mat = h5b_opencv::read(h5.get(), "/path/to/mat");
```


LICENSE
=======
Please see the file called [LICENSE](LICENSE)

<p align="center">
  <br/>
  <img src="h5_bridge/doc/figures/box-logo.png"/>
  <br/>
  Copyright &copy; 2020 Box Robotics, Inc.
</p>
