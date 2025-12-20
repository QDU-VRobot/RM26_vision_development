#ifndef INCLUD_SHOOTER_CLASS_HPP
#define INCLUD_SHOOTER_CLASS_HPP

#include "Armor.hpp"
#include <opencv2/core/types.hpp>
#include <array>
#include <cmath>

class Shooter
{

private:


cv::Point3d toward;

public:
    Shooter(const cv::Point3d& point):toward(point){}

// 返回值: {delta_pitch, delta_yaw}
    // 含义: 为了让 toward 重合 target，需要的旋转量
    std::array<double, 2> operator () (const ArmorPosi& armor)
    {
        // 1. 处理 target 向量 (归一化)
        cv::Point3d target = armor.posi;
        double t_norm = cv::norm(target);
        if (t_norm < 1e-6) return {0.0, 0.0};
        target /= t_norm;

        // 2. 计算两个向量的 Yaw (偏航角)
        // atan2(y, x) 算出的是向量在水平面投影与 X轴 的夹角
        double target_yaw = std::atan2(target.y, target.x);
        double toward_yaw = std::atan2(toward.y, toward.x);

        // 3. 计算两个向量的 Pitch (俯仰角)
        // asin(z) 算出的是向量与水平面的夹角
        double target_pitch = std::asin(target.z);
        double toward_pitch = std::asin(toward.z);

        // 4. 计算差值 (需要的旋转量)
        double delta_yaw   = target_yaw - toward_yaw;
        double delta_pitch = target_pitch - toward_pitch;

        // 5. 角度归一化 (关键步骤)
        // 处理跨越 ±180 度的情况，保证走最短路径
        // 例如：从 -170度 转到 +170度，应该是转 -20度，而不是 +340度
        delta_yaw = NormalizeAngle(delta_yaw);
        
        // Pitch 一般受限在 ±90 度以内，通常不需要归一化，但为了通用性可以加上
        // delta_pitch = NormalizeAngle(delta_pitch);

        return {delta_pitch, delta_yaw};
    }

    std::array<double, 2> operator () (const cv::Point3d& ShootPosi)
    {
        // 1. 处理 target 向量 (归一化)

        double t_norm = cv::norm(ShootPosi);
        if (t_norm < 1e-6) return {0.0, 0.0};
        cv::Point3d target = ShootPosi/t_norm;

        // 2. 计算两个向量的 Yaw (偏航角)
        // atan2(y, x) 算出的是向量在水平面投影与 X轴 的夹角
        double target_yaw = std::atan2(target.y, target.x);
        double toward_yaw = std::atan2(toward.y, toward.x);

        // 3. 计算两个向量的 Pitch (俯仰角)
        // asin(z) 算出的是向量与水平面的夹角
        double target_pitch = std::asin(target.z);
        double toward_pitch = std::asin(toward.z);

        // 4. 计算差值 (需要的旋转量)
        double delta_yaw   = target_yaw - toward_yaw;
        double delta_pitch = target_pitch - toward_pitch;

        // 5. 角度归一化 (关键步骤)
        // 处理跨越 ±180 度的情况，保证走最短路径
        // 例如：从 -170度 转到 +170度，应该是转 -20度，而不是 +340度
        delta_yaw = NormalizeAngle(delta_yaw);
        
        // Pitch 一般受限在 ±90 度以内，通常不需要归一化，但为了通用性可以加上
        // delta_pitch = NormalizeAngle(delta_pitch);

        return {delta_pitch, delta_yaw};
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