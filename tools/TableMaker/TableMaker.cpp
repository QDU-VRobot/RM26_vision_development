#include <cmath>
#include <cstddef>
#include <fstream>
#include <future>
#include <iostream>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// 表参数配置
constexpr double V = 19;     //弹速
constexpr bool TYPE = 1;     // 0为42mm(英雄),1为步兵


constexpr double MIN_PITCH = -(M_PI/6);    // 最小pitch限位 (rad)
constexpr double MAX_PITCH = (M_PI/4);     // 最大pitch限位 (rad)
constexpr double MAX_X = 10.0;        // 最大水平距离 (m)
constexpr double MIN_X = 0.0;         // 最小水平距离 (m)
constexpr double MAX_Y = 2.0;         // 最大高度 (m)
constexpr double MIN_Y = -1.0;        // 最小高度 (m)
constexpr double RESOLUTION = 0.01;  // 精度 (m)
constexpr double MAX_ERROR = 0.005;   // 允许误差 (m)
constexpr int ERROR_LEVEL = 5;        // 误差等级
constexpr double GUN = 0.10;          // 枪口到pitch轴电机的距离 (m)

constexpr double G = 9.8;        // 重力加速度 (m/s^2)
constexpr double STEP = 0.0001;  // RK4步长 (s)

// 弹丸状态
struct State
{
  double x, y, vx, vy;
  State(double x, double y, double vx, double vy) : x(x), y(y), vx(vx), vy(vy) {}
};

// 弹道解算器
class TrajectoryTableGenerator
{
 private:
  double v0_;        // 初速度
  double k_;         // 阻力系数
  double target_x_;  // 目标x坐标
  double target_y_;  // 目标y坐标
  double dt_;        // RK4步长

 public:
  TrajectoryTableGenerator(double v0, bool type, double target_x, double target_y,
                           double dt = STEP)
      : v0_(v0), target_x_(target_x), target_y_(target_y), dt_(dt)
  {
    if (type == 0)
    {
      // 英雄 (42mm弹丸)
      // 20度时空气密度、空气阻力系数、子弹直径、重量
      k_ = 1.205 * 0.40 * 0.0425 * 0.0425 / (2 * 0.0445);
    }
    else
    {
      // 步兵 (17mm弹丸)
      k_ = 1.205 * 0.47 * 0.0168 * 0.0168 / (2 * 0.0032);
    }
  }

  // 运动方程: dy/dt = f(t, y)
  std::vector<double> AirODE(const State& state)
  {
    double v = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    double ax = -k_ * v * state.vx;
    double ay = -G - k_ * v * state.vy;
    return {state.vx, state.vy, ax, ay};
  }

  // RK4 单步积分
  State RK4Step(const State& state, double h)
  {
    auto k1 = AirODE(state);
    State state1(state.x + 0.5 * h * k1[0], state.y + 0.5 * h * k1[1],
                 state.vx + 0.5 * h * k1[2], state.vy + 0.5 * h * k1[3]);

    auto k2 = AirODE(state1);
    State state2(state.x + 0.5 * h * k2[0], state.y + 0.5 * h * k2[1],
                 state.vx + 0.5 * h * k2[2], state.vy + 0.5 * h * k2[3]);

    auto k3 = AirODE(state2);
    State state3(state.x + h * k3[0], state.y + h * k3[1], state.vx + h * k3[2],
                 state.vy + h * k3[3]);

    auto k4 = AirODE(state3);

    State new_state(state.x, state.y, state.vx, state.vy);
    new_state.x += h * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6.0;
    new_state.y += h * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6.0;
    new_state.vx += h * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6.0;
    new_state.vy += h * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]) / 6.0;

    return new_state;
  }

  // 二分法搜索pitch
  std::vector<double> SolvePitch(double error)
  {
    double t_b = GUN / v0_;
    double pitch_top = MAX_PITCH;
    double pitch_low = MIN_PITCH;

    while ((pitch_top - pitch_low) > 0.001)
    {
      double count = 0;
      double pitch_binary = (pitch_top + pitch_low) / 2;
      double x_b = -GUN * std::cos(pitch_binary);
      double y_b = -GUN * std::sin(pitch_binary);
      double x_to_gun = target_x_ + x_b;
      double y_to_gun = target_y_ + y_b;
      State state(0, 0, v0_ * std::cos(pitch_binary), v0_ * std::sin(pitch_binary));

      while (state.y >= MIN_Y - 1)
      {
        state = RK4Step(state, dt_);
        count++;

        if (std::pow(state.x - x_to_gun, 2) + std::pow(state.y - y_to_gun, 2) <=
            std::pow(error, 2))
        {
          return {pitch_binary, count * dt_ + t_b,
                  std::sqrt(state.vx * state.vx + state.vy * state.vy)};
        }

        if (state.x >= x_to_gun)
        {
          if (state.y > y_to_gun)
          {
            pitch_top = pitch_binary;
          }
          else
          {
            pitch_low = pitch_binary;
          }
          break;
        }
        else if (state.y < MIN_Y - 1 && state.x < x_to_gun)
        {
          pitch_low = pitch_binary;
          break;
        }
      }
    }
    return {NAN, NAN, NAN};
  }

  // 多级误差搜索
  std::vector<double> SolvePitchLevel(int error_level)
  {
    auto ge = SolvePitch(MAX_ERROR);
    if (std::isnan(ge[0]))
    {
      return {NAN, NAN, NAN};
    }
    for (int i = 1; i <= error_level; i++)
    {
      ge = SolvePitch(MAX_ERROR / error_level * i);
      if (!std::isnan(ge[0]))
      {
        return ge;
      }
    }
    return {NAN, NAN, NAN};
  }
};

using TableData = std::vector<std::vector<std::vector<double>>>;

// 解算单行
static TableData solve_rows(double start_x, size_t num_rows, double v0, bool bullet_type)
{
  double x = start_x;
  size_t y_dim = static_cast<size_t>(std::round((MAX_Y - MIN_Y) / RESOLUTION + 1));
  TableData table;
  table.reserve(num_rows);

  for (size_t i = 0; i < num_rows; i++, x += RESOLUTION)
  {
    double y = MIN_Y;
    std::vector<std::vector<double>> row;
    row.reserve(y_dim);
    for (size_t j = 0; j < y_dim; j++, y += RESOLUTION)
    {
      TrajectoryTableGenerator solve(v0, bullet_type, x, y);
      std::vector<double> ge = solve.SolvePitchLevel(ERROR_LEVEL);
      row.push_back(ge);
    }
    table.push_back(std::move(row));
  }
  return table;
}

// 输出表格解的情况
template <typename T>
static std::ostream& operator<<=(std::ostream& os, const std::vector<T>& v)
{
  for (const T& x : v)
  {
    os <<= x;
  }
  os << "\n";
  return os;
}

template <>
std::ostream& operator<<=(std::ostream& os, const std::vector<double>& v)
{
  return os << (std::isnan(v[0]) ? ' ' : '.');
}

static void build_table(double v0, bool bullet_type, const std::string& output_prefix)
{
  TableData table;
  std::ios_base::sync_with_stdio(false);
  std::queue<std::future<TableData>> futures;
  size_t threads = std::thread::hardware_concurrency();
  if (threads == 0)
  {
    threads = 16;
  }

  size_t total_rows = static_cast<size_t>(std::round((MAX_X - MIN_X) / RESOLUTION + 1));
  table.reserve(total_rows);

  std::cerr << "开始生成弹道查找表..." << '\n';
  std::cerr << "弹速: " << v0 << " m/s" << '\n';
  std::cerr << "弹丸类型: " << (bullet_type ? "17mm" : "42mm") << '\n';
  std::cerr << "使用 " << threads << " 个线程" << '\n';
  std::cerr << "总行数: " << total_rows << '\n';

  double current_x = MIN_X;
  size_t rows_processed = 0;

  while (rows_processed < total_rows)
  {
    size_t batch_size = std::min(threads, total_rows - rows_processed);

    for (size_t i = 0; i < batch_size; ++i)
    {
      futures.push(
          std::async(std::launch::async, solve_rows, current_x, 1, v0, bullet_type));
      current_x += RESOLUTION;
    }

    while (!futures.empty())
    {
      TableData single_row_table = futures.front().get();
      futures.pop();
      table.insert(table.end(), std::make_move_iterator(single_row_table.begin()),
                   std::make_move_iterator(single_row_table.end()));
    }

    rows_processed += batch_size;
    std::cerr << "进度: " << rows_processed << " / " << total_rows << " 行已完成" << '\n';
  }

  std::cerr << "计算完成。" << '\n';
  (std::cerr <<= table) << '\n';

  // 二进制文件写入
  std::string output_filename =
      output_prefix + "_" + std::to_string(static_cast<int>(MAX_X)) + "_table.bin";
  std::ofstream file_out(output_filename.c_str(), std::ios::out | std::ios::binary);
  if (!file_out)
  {
    std::cerr << "错误: 无法打开文件进行写入: " << output_filename << '\n';
    return;
  }

  struct Cell
  {
    float pitch;
    float t;
    float v;
  };

  for (const auto& row : table)
  {
    for (const auto& ge : row)
    {
      Cell cell_to_write;
      if (ge.empty() || std::isnan(ge[0]))
      {
        cell_to_write = {NAN, NAN, NAN};
      }
      else
      {
        cell_to_write = {static_cast<float>(ge[0]), static_cast<float>(ge[1]),
                         static_cast<float>(ge[2])};
      }
      file_out.write(reinterpret_cast<const char*>(&cell_to_write), sizeof(Cell));
    }
  }

  file_out.close();
  std::cerr << "二进制查找表已成功生成到 " << output_filename << '\n';
}

int main(int argc, char* argv[])
{
  double v0 = V;        // 默认弹速
  bool bullet_type = TYPE;  // 默认42mm (英雄)
  std::string prefix = bullet_type ? "infantry" : "hero";

  if (argc >= 2)
  {
    v0 = std::stod(argv[1]);
  }
  if (argc >= 3)
  {
    bullet_type = std::stoi(argv[2]);
    prefix = bullet_type ? "infantry" : "hero";
  }
  if (argc >= 4)
  {
    prefix = argv[3];
  }

  std::cerr << "========================================" << '\n';
  std::cerr << "弹道查找表生成器" << '\n';
  std::cerr << "用法: " << argv[0] << " [弹速] [弹丸类型:0=42mm,1=17mm] [输出前缀]"<<"解算弹丸类型: "<< bullet_type
            << '\n';
  std::cerr << "========================================" << '\n';

  build_table(v0, bullet_type, prefix);
  return 0;
}