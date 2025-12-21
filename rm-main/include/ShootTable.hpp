#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>


//=============================================================================
// 弹道查找表类
//=============================================================================
class ShootTable
{
 public:
  struct TableConfig
  {
    double max_x, min_x, max_y, min_y, resolution;
    size_t x_dim, y_dim;
    std::string filename;

    TableConfig(double max_x, double min_x, double max_y, double min_y, double resolution,
                std::string filename)
        : max_x(max_x),
          min_x(min_x),
          max_y(max_y),
          min_y(min_y),
          resolution(resolution),
          x_dim(static_cast<size_t>((max_x - min_x) / resolution) + 1),
          y_dim(static_cast<size_t>((max_y - min_y) / resolution) + 1),
          filename(std::move(filename))
    {
    }
  };

  struct Cell
  {
    float pitch;
    float t;
    float v;
  };

  explicit ShootTable(const TableConfig& config)
      : MAX_X(config.max_x),
        MIN_X(config.min_x),
        MAX_Y(config.max_y),
        MIN_Y(config.min_y),
        RESOLUTION(config.resolution),
        X_DIM(config.x_dim),
        Y_DIM(config.y_dim),
        filename_(config.filename)
  {
  }

  ~ShootTable() = default;

  // 查表获取弹道参数
  Cell Check(float x, float y) const
  {
    if (!init_)
    {
      return {NAN, NAN, NAN};
    }

    // 边界检查
    float adjusted_x = x;
    float adjusted_y = y;

    if (adjusted_x < MIN_X || adjusted_x > MAX_X || adjusted_y < MIN_Y ||
        adjusted_y > MAX_Y)
    {
      return {NAN, NAN, NAN};
    }

    size_t xc = static_cast<size_t>(std::round((adjusted_x - MIN_X) / RESOLUTION));
    size_t yc = static_cast<size_t>(std::round((adjusted_y - MIN_Y) / RESOLUTION));
    xc = std::min(xc, X_DIM - 1);
    yc = std::min(yc, Y_DIM - 1);

    Cell ge = table_[xc * Y_DIM + yc];

    return {ge.pitch, ge.t, ge.v};
  }

  // 初始化：从二进制文件加载表
  bool Init()
  {
    table_.resize(X_DIM * Y_DIM);

    std::ifstream file_in(filename_, std::ios::in | std::ios::binary);

    if (!file_in)
    {
      std::cerr << "[TrajectoryTable] 错误: 无法打开文件 " << filename_
                << "，使用默认弹道解算" << '\n';
      init_ = false;
      return false;
    }

    const std::size_t BYTES_TO_READ = X_DIM * Y_DIM * sizeof(Cell);

    file_in.read(reinterpret_cast<char*>(table_.data()),
                 static_cast<std::streamsize>(BYTES_TO_READ));

    if (!file_in || file_in.gcount() != static_cast<std::streamsize>(BYTES_TO_READ))
    {
      std::cerr << "[TrajectoryTable] 错误: "
                   "读取数据失败或文件大小不匹配，使用默认弹道解算"
                << '\n';
      init_ = false;
      return false;
    }

    file_in.close();
    init_ = true;
    std::cout << "[TrajectoryTable] 弹道查找表加载成功: " << filename_ << '\n';
    return true;
  }

  bool IsInit() const { return init_; }

  // Getter
  double GetMinX() const { return MIN_X; }
  double GetMaxX() const { return MAX_X; }
  double GetMinY() const { return MIN_Y; }
  double GetMaxY() const { return MAX_Y; }

 private:
  const double MAX_X;
  const double MIN_X;
  const double MAX_Y;
  const double MIN_Y;
  const double RESOLUTION;

  const size_t X_DIM;
  const size_t Y_DIM;

  bool init_ = false;
  std::string filename_;
  std::vector<Cell> table_;
};

//=============================================================================
// 弹道解算主类
//=============================================================================


#pragma once