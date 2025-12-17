#ifndef TARGET_HPP
#define TARGET_HPP

#include "Armor.hpp"

#include <array>
#include <opencv2/core/types.hpp>
#include <vector>

class Target
{
public:
    Target(double radius = 200.0,double radius2 = 200.0);

    //观测
    void correct(const std::array<ArmorPosi,2>& armors);

    ArmorPosi move(const ArmorPosi& armor, const cv::Point3d& v, double w, double dt);//mm/s, rad/s

    std::array<ArmorPosi, 4> wholeTargetPosi(const ArmorPosi& armor);

private:

//半径
    double r1, r2; //目标半径mm
    std::array<double,1000> r1_set;//记录目标半径的数组
    std::array<double,1000> r2_set;//记录目标半径的数组
    double r1_sum = 0.0;//目标半径和
    double r2_sum = 0.0;//目标半径和


    int r_num = 0;//记录目标半径的数量

//中心转轴方向
    cv::Point3d axis = cv::Point3d(0,0,1);//单位向量
};
#endif // TARGET_HPP