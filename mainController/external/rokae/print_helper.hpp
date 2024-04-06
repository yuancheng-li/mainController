/**
 * @file print_helper.hpp
 * @brief 打印接口调用结果
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef LIBROKAEEXAMPLE_EXAMPLE_CPP_PRINT_HELPER_HPP_
#define LIBROKAEEXAMPLE_EXAMPLE_CPP_PRINT_HELPER_HPP_

#include <iostream>
#include <array>
#include <vector>
#include <iterator>
#include "rokae/data_types.h"

template <class T, size_t S>
inline std::ostream &operator<<(std::ostream &os, const std::array<T,S> &arr) {
  os << "[ ";
  std::copy(arr.cbegin(), arr.cend() - 1, std::ostream_iterator<T>(os, ", "));
  std::copy(arr.cend() - 1, arr.cend(), std::ostream_iterator<T>(os));
  os << " ]";
  return os;
}

template <class T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &arr) {
  os << "[ ";
  std::copy(arr.cbegin(), arr.cend() - 1, std::ostream_iterator<T>(os, ", "));
  std::copy(arr.cend() - 1, arr.cend(), std::ostream_iterator<T>(os));
  os << " ]";
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const rokae::Frame &frame) {
  os << "[ X: " << frame.trans[0] << " Y: " << frame.trans[1] << " Z: " << frame.trans[2] <<
  " A: " << frame.rpy[0] << " B: " << frame.rpy[1] << " C: " << frame.rpy[2] << " ]";
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const rokae::Load &load) {
  os << "质量: " << load.mass << "kg, 重心 X: " << load.cog[0] << " Y: " << load.cog[1] << " Z: " << load.cog[2] <<
  ", 惯量 ix: " << load.inertia[0] << " iy: " << load.inertia[1] << " iz: " << load.inertia[2];
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const rokae::Toolset &toolset) {
  os << "手持 - " << toolset.end << "\n外部 - " << toolset.ref <<
  "\n负载 - " << toolset.load;
  return os;
}

template <typename... Args>
void print(std::ostream &os, Args&&... args) {
  ((os << ' '<< std::forward<Args>(args)), ...) << std::endl;
}

#endif //LIBROKAEEXAMPLE_EXAMPLE_CPP_PRINT_HELPER_HPP_
