#ifndef INCLUD_SHOOTER_CLASS_HPP
#define INCLUD_SHOOTER_CLASS_HPP

#include "Armor.hpp"
#include "ShootTable.hpp"
#include <opencv2/core/types.hpp>
#include <array>
#include <cmath>

class Shooter
{

private:

const cv::Point3d toward;
ShootTable table;
const double toward_pitch,toward_yaw;
public:
    Shooter(const cv::Point3d& point,ShootTable::TableConfig config):
            toward(point), 
            table(config),
            toward_pitch(std::asin(toward.z)),
            toward_yaw(std::atan2(toward.y, toward.x))
            { table.Init();}

    /**
    @brief 计算射击目标，枪口需要转动的pitch和yaw(单位：rad)
    @param ShootPosi 射击位置的坐标(单位：m)
    @return a array that array[0]: pitch, array[1]: yaw(单位：rad)
    */
    std::array<double, 2> operator () (const cv::Point3d& ShootPosi)
    {

        // 1. 计算target向量 Yaw (偏航角)

        //处理 target 向量 (归一化)
        double t_norm = cv::norm(ShootPosi);
        if (t_norm < 1e-6) return {0.0, 0.0};
        const cv::Point3d target = ShootPosi/t_norm;

        // atan2(y, x) 算出的是向量在水平面投影与 X轴 的夹角
        double shoot_yaw = std::atan2(target.y, target.x);


        // 2. 计算target向量 Pitch (俯仰角）
        //通过查表进行计算，算出的是向量与水平面的夹角
        
        //计算水平距离
        float distance = std::sqrt(ShootPosi.x * ShootPosi.x + ShootPosi.y * ShootPosi.y);
        
        //查表
        auto ans = table.Check(distance, (float)ShootPosi.z);
        double shoot_pitch = ans.pitch;


        // 3. 计算差值 (需要的旋转量)
        double delta_yaw   = shoot_yaw - this->toward_yaw;
        double delta_pitch = shoot_pitch - this->toward_pitch;

        // 5. 角度归一化 (关键步骤)
        // 处理跨越 ±180 度的情况，保证走最短路径
        // 例如：从 -170度 转到 +170度，应该是转 -20度，而不是 +340度
        delta_yaw = NormalizeAngle(delta_yaw);
        
        // Pitch 一般受限在 ±90 度以内，通常不需要归一化，但为了通用性可以加上
        // delta_pitch = NormalizeAngle(delta_pitch);

        return {delta_pitch, delta_yaw};
    }

    /**
    @brief 计算从发射到击中目标的飞行时间
    @param Posi 射击位置的坐标(单位：m)
    @return 返回弹丸需要飞行的时间(单位：s)
    */
    float FlyTime(const cv::Point3d& Posi)
    {
        //计算水平距离
        float distance = std::sqrt(Posi.x * Posi.x + Posi.y * Posi.y);

        auto ans = table.Check(distance, (float)Posi.z);
        
        return ans.t;
    }


// 辅助函数：将角度限制在 [-PI, PI] 之间
    double NormalizeAngle(double angle) 
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

};



#endif