/**
 * ************************************************************************
 * 
 * @file timer.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 计时器, 构造时开始计时, 析构时结束并打印时间
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2024 
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once

#include <iostream>
#include <chrono>
#include <glog/logging.h>

class Timer {
public:
  Timer(const char* name) : m_Name(name), m_Stop(false) {
    m_StartTimepoint = std::chrono::high_resolution_clock::now();
  }

  void Stop() {
    auto endTimePoint = std::chrono::high_resolution_clock::now();

    long long start = std::chrono::time_point_cast<std::chrono::milliseconds>(m_StartTimepoint).time_since_epoch().count();
    long long end = std::chrono::time_point_cast<std::chrono::milliseconds>(endTimePoint).time_since_epoch().count();

    LOG(INFO) << m_Name << ":" << (end - start) << "ms\n";
    
    if ((end -start) > 50)
      LOG(WARNING) << "PROCESS TIME OVER 50MS!!!";

    m_Stop = true;
  }

  ~Timer() {
    if (!m_Stop)
      Stop();
  }

private:
  const char* m_Name;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_StartTimepoint;
  bool m_Stop;
};