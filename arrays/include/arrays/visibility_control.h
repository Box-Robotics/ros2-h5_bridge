// -*- c++ -*-
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARRAYS__VISIBILITY_CONTROL_H_
#define ARRAYS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARRAYS_EXPORT __attribute__ ((dllexport))
    #define ARRAYS_IMPORT __attribute__ ((dllimport))
  #else
    #define ARRAYS_EXPORT __declspec(dllexport)
    #define ARRAYS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ARRAYS_BUILDING_DLL
    #define ARRAYS_PUBLIC ARRAYS_EXPORT
  #else
    #define ARRAYS_PUBLIC ARRAYS_IMPORT
  #endif
  #define ARRAYS_PUBLIC_TYPE ARRAYS_PUBLIC
  #define ARRAYS_LOCAL
#else
  #define ARRAYS_EXPORT __attribute__ ((visibility("default")))
  #define ARRAYS_IMPORT
  #if __GNUC__ >= 4
    #define ARRAYS_PUBLIC __attribute__ ((visibility("default")))
    #define ARRAYS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ARRAYS_PUBLIC
    #define ARRAYS_LOCAL
  #endif
  #define ARRAYS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ARRAYS__VISIBILITY_CONTROL_H_
