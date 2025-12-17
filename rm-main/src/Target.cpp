#include "../include/Target.hpp"
#include <iostream>
#include <opencv2/core/types.hpp>


Target::Target(double radius, double radius2)
    : r1(radius), r2(radius2){}


void Target::correct(const std::array<ArmorPosi,2>& armors)
{
    if(this->r_num >= 1000) { r_num = 0;return;}
    
    //计算中心轴向量
    this-> axis = armors[0].toward.cross(armors[1].toward);
    this->axis = this->axis / cv::norm(this->axis);//单位化

    double cos0 = axis.dot(cv::Point3d(0,0,1))/(cv::norm(axis)*1.0);
    if(cos0 < 0) axis = -axis;

    //计算目标半径并记录

    cv::Point3d n1 = axis.cross(armors[0].toward);
    cv::Point3d n2 = axis.cross(armors[1].toward);
    n1 = n1 / cv::norm(n1);//单位化
    n2 = n2 / cv::norm(n2);//单位化

    cv::Point3d AB = armors[1].posi - armors[0].posi;

    double b1 = AB.dot(n1);
    double b2 = AB.dot(n2);

    double k = n1.dot(n2);
    
    double deno = 1 - k*k;

//计算r
    this->r1_set[r_num] = ( b1 - b2 * k ) / deno;
    this->r2_set[r_num] = ( b1 * k - b2 ) / deno;
    // std::cout << "Target radius updated: r1 = " << this->r1_set[r_num] << " mm, r2 = " << this->r2_set[r_num] << " mm" << std::endl;
    std::cout << "Target axis: = " << this->axis <<"\n";
    this->r1_sum += this->r1_set[r_num];
    this->r2_sum += this->r2_set[r_num];

    r_num++;

//更新目标半径
    this->r1 = this->r1_sum / r_num;
    this->r2 = this->r2_sum / r_num;

}



