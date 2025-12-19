#ifndef SOLVER_CLASS_INCLUDE
#define SOLVER_CLASS_INCLUDE
#include "Armor.hpp"
#include "string"
#include "opencv2/opencv.hpp"
#include <deque>
#include <opencv2/core/base.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/quaternion.hpp>
#include <vector>
class Solver
{
public:

    Solver(std::string config_path);
    
    //解算传入的所有装甲板并返回
    std::vector<ArmorPosi> operator () (const std::vector<Armor>& armors);
    
    //解算传入的所有装甲板但只返回距离先验装甲板最近的装甲板
    ArmorPosi operator () (const std::deque<Armor>& armors,const Armor& armor);
    
    //解算单个装甲板的位置
    ArmorPosi operator () (const Armor& armor);

    //坐标系变换
    void ConverToWorld(ArmorPosi& armor_posi, const cv::Quatd& gripper_to_world);
    void ConverToWorld(std::vector<ArmorPosi>& armor_posi, const cv::Quatd& gripper_to_world);

    void ansShow(const cv::Point3d& posi,cv::Mat& image);
    void ansShow(const ArmorPosi& armor,cv::Mat& image);

private:
    cv::Mat_<double> cameraMatrix;
    cv::Mat_<double> distCoeffs;

    cv::Mat_<double> R_Cam_to_gripper;
    cv::Mat_<double> T_Cam_to_gripper;

    cv::Mat_<double> BigArmorCenter;
    cv::Mat_<double> SmallArmorCenter;
    std::vector<cv::Point3f> objectBigArmorP{{0,0,0},{230,0,0},{230,55,0},{0,55,0}};
    std::vector<cv::Point3f> objectSmallArmorP{{0,0,0},{135,0,0},{135,55,0},{0,55,0}};
};

#endif