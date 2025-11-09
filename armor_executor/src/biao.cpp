#include "armor_executor/biao.hpp"

Point3d checkbiao(float x, float y, float x_bias, float y_bias,
                  float pitch_bias, float t_bias) {

  size_t xc = std::round((x + x_bias - MIN_X) * 100 / RESOLUTION) / 100;
  size_t yc = std::round((y + y_bias - MIN_Y) * 100 / RESOLUTION) / 100;
  Point3d ge = biao[xc][yc];

  std::cout << xc << yc << std::endl;
  return {ge.pitch + pitch_bias, ge.t + t_bias, ge.v};
}

// int main() {
//   std::cout << biao[1].size() << std::endl;
//   std::cout << biao.size() << std::endl;
//   std::array<double, 3> ge = checkbiao(1, 1);
//   std::cout << ge[0] << std::endl;
//   return 0;
// }
