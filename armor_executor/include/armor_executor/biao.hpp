#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

constexpr double MAX_X = 6.0;
constexpr double MIN_X = 0.0;
constexpr double MAX_Y = 1.0;
constexpr double MIN_Y = -1;
constexpr double RESOLUTION = 0.01;

struct Point3d {
  float pitch, t, v;
};

Point3d checkbiao(float x, float y, float x_bias = 0, float y_bias = 0,
                  float pitch_bias = 0.00, float t_bias = 0);
constexpr size_t col = (MAX_Y - MIN_Y) / RESOLUTION + 1;
constexpr size_t row = (MAX_X - MIN_X) / RESOLUTION + 1;
extern const std::array<std::array<Point3d, col>, row> biao;
