#include <cmath>
#include <cstddef>
#include <future>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

constexpr double MIN_PITCH = -0.6;
constexpr double MAX_PITCH = 1.2;
constexpr double MAX_X = 6.0;
constexpr double MIN_X = 0.0;
constexpr double MAX_Y = 1.0;
constexpr double MIN_Y = -1;
constexpr double RESOLUTION = 0.01;
constexpr double MAX_ERROR = 0.05;
constexpr int ERROR_LEVEL = 4;
constexpr double GUN = 0.30;

constexpr double g = 9.8;
constexpr double STEP = 0.0001;

using Table = std::vector<std::vector<std::vector<double>>>;

struct State {
  double x, y, vx, vy;
  State(double x, double y, double vx, double vy)
      : x(x), y(y), vx(vx), vy(vy) {}
};

class SolveTrajectory {
private:
  double v0_;
  double k_;        // 阻力系数
  double target_x_; // 目标x，y坐标 ,相对小车pitch轴电机(小车中心点)
  double target_y_;
  double dt_; // RK4步长 (s)

public:
  SolveTrajectory(double v0, bool type, double target_x, double target_y,
                  double dt = STEP)
      : v0_(v0), target_x_(target_x), target_y_(target_y), dt_(dt) {
    if (type == 0) {
      k_ = 1.205 * 0.40 * 0.0425 * 0.0425 / (2 * 0.0445);
    } else if (type == 1) {
      k_ = 1.205 * 0.47 * 0.0168 * 0.0168 / (2 * 0.0032);
    }
  }

  // 运动方程: dy/dt = f(t, y)
  std::vector<double> airODE(const State &state) {
    double v = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    double ax = -k_ * v * state.vx;      // dvx/dt
    double ay = -g - k_ * v * state.vy;  // dvy/dt
    return {state.vx, state.vy, ax, ay}; // [dx/dt, dy/dt, dvx/dt, dvy/dt]
  }

  // RK4 单步积分
  State rk4_step(const State &state, double h) {
    auto k1 = airODE(state);
    State state1(state.x + 0.5 * h * k1[0], state.y + 0.5 * h * k1[1],
                 state.vx + 0.5 * h * k1[2], state.vy + 0.5 * h * k1[3]);

    auto k2 = airODE(state1);
    State state2(state.x + 0.5 * h * k2[0], state.y + 0.5 * h * k2[1],
                 state.vx + 0.5 * h * k2[2], state.vy + 0.5 * h * k2[3]);

    auto k3 = airODE(state2);
    State state3(state.x + h * k3[0], state.y + h * k3[1], state.vx + h * k3[2],
                 state.vy + h * k3[3]);

    auto k4 = airODE(state3);

    State new_state(state.x, state.y, state.vx, state.vy);
    new_state.x += h * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6.0;
    new_state.y += h * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6.0;
    new_state.vx += h * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6.0;
    new_state.vy += h * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]) / 6.0;

    return new_state;
  }

  //   double isPitch(double pitch) {

  //     }
  //     return 0;
  //   }

  // 单次目标弹道解算，实际解算从枪口计算，这里考虑枪管长度做补偿
  std::vector<double> solvePitch(double pitch, double error) {
    double count = 0;
    double t_b = GUN / v0_;
    while (pitch < MAX_PITCH) {
      count = 0;
      double x_b = -GUN * std::cos(pitch);
      double y_b = -GUN * std::sin(pitch);
      double x_togun = target_x_ + x_b;
      double y_togun = target_y_ + y_b;
      State state(0, 0, v0_ * std::cos(pitch), v0_ * std::sin(pitch));
      while (state.x < x_togun + error) {
        state = rk4_step(state, dt_);
        count++;
        //   double x = std::round(state.x * 1000.0) / 1000.0;
        //   double y = std::round(state.y * 1000.0) / 1000.0;
        //   if (std::fabs(state.x - target_x_) < 0.01) {
        //     std::cout << state.x << "和" << state.y << std::endl;
        //   }

        if (pow(state.x - x_togun, 2) + pow(state.y - y_togun, 2) <=
            pow(error, 2)) {
          return {pitch, count * STEP + t_b,
                  std::sqrt(state.vx * state.vx + state.vy * state.vy)};
        }
      }
      pitch += 0.01;
    }
    return {NAN, NAN, NAN};
  }

  // 对solvePitch的优化，考虑多级误差以保证精确和有解
  std::vector<double> solvePitch_level(int error_level,
                                       double pitch0 = MIN_PITCH) {
    if (std::isnan(solvePitch(pitch0, MAX_ERROR)[0]))
      return {NAN, NAN, NAN};
    for (size_t i = 0; i <= error_level; i++) {
      auto ge = solvePitch(pitch0, MAX_ERROR / error_level * i);
      if (!std::isnan(ge[0]))
        return ge;
    }
    return {NAN, NAN, NAN};
  }

  // 给角度解位置
  std::vector<double> solveHeightAndLength(double pitch) {
    double max_heigh = 0;
    double max_length = 0;
    State state(0, 0, v0_ * std::cos(pitch), v0_ * std::sin(pitch));
    while (state.y > -0.5) {
      state = rk4_step(state, dt_);
      if (max_heigh < state.y)
        max_heigh = state.y;
      if (max_length < state.x)
        max_length = state.x;
    }
    return {max_heigh, max_length};
  }
};

// 解算多行
Table solve_rows(double s, size_t xc) {
  double pitch0 = MIN_PITCH;
  double x = s;
  double y;
  size_t yc = std::round((MAX_Y - MIN_Y) / RESOLUTION) + 1;
  Table biao;
  biao.reserve(xc);
  // std::vector<std::vector<std::vector<double>>>(
  //     qu, std::vector<std::vector<double>>((MAX_Y - MIN_Y) / JINGDU,
  //                                          std::vector<double>(3, 0.0)));
  for (size_t i = 0; i < xc; i++, x += RESOLUTION) {
    y = MIN_Y;
    std::vector<std::vector<double>> row;
    row.reserve(yc);
    for (size_t j = 0; j < yc; j++, y += RESOLUTION) {
      SolveTrajectory solve = SolveTrajectory(11.8, 0, x, y);
      std::vector<double> ge = solve.solvePitch_level(ERROR_LEVEL, pitch0);
      if (!std::isnan(ge[0]))
        pitch0 = ge[0];
      row.push_back(ge);
    }
    pitch0 = MIN_PITCH;
    biao.push_back(std::move(row));
    std::cerr << '[' << x << '/' << s + (xc - 1) * RESOLUTION << ']'
              << std::endl;
  }
  // std::cout << "right------------" << i << std::endl;
  return biao;
}

// 输出得到的表，按array的格式来
template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << '{';
  for (size_t i = 0; i < v.size(); ++i) {
    if (i > 0)
      os << ',';
    os << v[i];
  }
  return os << '}';
}

template <typename T>
std::ostream &operator<<(std::ostream &os,
                         const std::vector<std::vector<T>> &v) {
  os << "{{";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i > 0)
      os << ',';
    os << v[i];
  }
  return os << "}}";
}

std::ostream &operator<<(std::ostream &os, const Table &v) {
  os << "{{\n";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i > 0)
      os << ",\n";
    os << "  " << v[i];
  }
  return os << "\n}}";
}

// 输出表格解的情况，检查是否有无解的情况
template <typename T>
std::ostream &operator<<=(std::ostream &os, const std::vector<T> &v) {
  for (T x : v)
    os <<= x;
  os << "\n";
  return os;
}

template <>
std::ostream &operator<<=(std::ostream &os, const std::vector<double> &v) {
  return os << (std::isnan(v[0]) ? ' ' : '.');
}

// 多线程提高效率
int main() {
  Table biao;
  std::ios::sync_with_stdio(false);
  std::queue<std::future<Table>> futures;
  size_t threads = std::thread::hardware_concurrency();
  if (!threads)
    threads = 16;
  size_t total = std::round((MAX_X - MIN_X) / RESOLUTION) + 1,
         count = total / threads, remaining = total % threads;
  if (!count)
    threads = remaining;
  std::cerr << "Threads: " << threads << " Count: " << count << " ... "
            << remaining << std::endl;
  double x = MIN_X;
  for (size_t i = 0; i < threads; i++) {
    futures.push(std::async(solve_rows, x, count + !!remaining));
    if (remaining > 0) {
      x += RESOLUTION * (count + !!remaining);
      remaining--;
    }
  }
  biao.reserve(count);
  while (futures.size()) {
    Table rows = futures.front().get();
    futures.pop();
    std::move(rows.begin(), rows.end(), std::back_inserter(biao));
  }
  std::cerr << "行数x:" << biao.size() << std::endl;
  std::cerr << "列数y:" << biao[0].size() << std::endl;
  (std::cerr <<= biao) << std::endl;
  (std::cout << biao) << std::endl;
  return 0;
}

// int main() {
//   double x = 0;
//   double y = 0;
//   double pitchdu = 0;
//   SolveTrajectory solve(11.3, 0, x, y);

//   // std::vector<double> ge1 = solve.solvePitch();
//   // double pitch = ge1[0];
//   // double time = ge1[1];
//   // double hit_v = ge1[2];
//   // std::cout << "(" << pitch * 180 / 3.1416 << ", " << time << ", " <<
//   hit_v
//   //           << ")" << std::endl;

//   std::vector<double> ge2 = solve.solveHeightAndLength(pitchdu * 3.1416 /
//   180); double max_height = ge2[0]; double max_length = ge2[1]; std::cout <<
//   "(" << max_height << "," << max_length << ")" << std::endl;

//   return 0;
// }
