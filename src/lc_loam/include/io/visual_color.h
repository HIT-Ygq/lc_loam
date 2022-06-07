#pragma once

#include <array>
#include <cmath>

#include "ros/ros.h"
#include <std_msgs/ColorRGBA.h>
#include "glog/logging.h"

std::array<float, 3> HsvToRgb(const float h, const float s, const float v);
std::array<float, 3> GetColor(int id);

::std_msgs::ColorRGBA ToMessage(const std::array<float, 3> &color);