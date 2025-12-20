#include "HikCamera.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <iomanip>    // 用于格式化时间字符串
#include <sstream>    // 用于构建字符串

// 定义一个结构体，用于在回调函数中传递数据
struct MouseParams {
    cv::Mat frame;
    cv::Point mousePoint;
};

// 鼠标回调函数
void onMouse(int event, int x, int y, int flags, void* userdata) {
    MouseParams* params = (MouseParams*)userdata;
    if (event == cv::EVENT_MOUSEMOVE) {
        // 记录当前鼠标坐标
        params->mousePoint = cv::Point(x, y);
    }
}

// 输入参数：像素点, 内参, 畸变系数, 深度
cv::Point3f getCameraCoordinates(const cv::Point2f& pixelPoint, 
                                 const cv::Mat& K, 
                                 const cv::Mat& distCoeffs, 
                                 float depth) {
    // 1. 准备像素点
    std::vector<cv::Point2f> points = { pixelPoint };
    std::vector<cv::Point2f> undistortedPoints;

    // 2. 去畸变并转换到归一化坐标系 (z=1 的平面)
    // 注意：P 参数如果不传，返回的是 (x/z, y/z)
    cv::undistortPoints(points, undistortedPoints, K, distCoeffs, cv::noArray(), cv::noArray());

    // 3. 这里的 x_norm 和 y_norm 是归一化坐标 (x/z, y/z)
    float x_norm = undistortedPoints[0].x;
    float y_norm = undistortedPoints[0].y;

    // 4. 根据深度 z_c 还原三维坐标
    float z_c = depth;
    float x_c = x_norm * z_c;
    float y_c = y_norm * z_c;

    return cv::Point3f(x_c, y_c, z_c);
}
MouseParams params;
std::string winName = "Real-time Pixel Viewer";
int main() {

    // 打开摄像头
    io::HikCamera Hik(1,10,3);
    io::HikCamera::ImageData frames; 


    double a[9] = {1802.872008809263, 0, 683.3658853864414,
 0, 1802.113248135188, 540.8637357540931,
 0, 0, 1};
    double b[5] = {-0.1086133938847445,
 0.5064197841529977,
 -0.0008509175199885377,
 -0.003021875590926957,
 -1.141598985774171};
    cv::Mat_<double> camera_matrix(3,3,a);
    cv::Mat_<double> distortion_coeffs(5,1,b);


    double r[9] = {-0.0110160746370467, 0.0179947500041167, -0.9997773927589484,
                   0.999805789595447, -0.01614115599515337, -0.01130690826729929,
                   -0.01634102784453971, -0.999707783332424, -0.01781344305726771};
    double t[3] = {-176.3554534276337,
 -7.489853758266302,
 47.03696352843404};
    cv::Mat_<double> R(3,3,r);
    cv::Mat_<double> T(3,1,t);

    float depth  = 1370;
    // std::cin >> depth;
    

    std::cout << "摄像头已成功打开。按 '空格键' 截图，按 'ESC' 退出。" << std::endl;

    //创建一个窗口用于显示
    cv::namedWindow(winName);
    cv::setMouseCallback(winName, onMouse, &params);

    while (true) 
    {
        // 从摄像头捕获一帧
        Hik.read(frames);
        cv::Mat frame = frames.image;

        // 复制一份用于显示，避免在原始帧上留下永久文字
        cv::Mat displayFrame = frame.clone();

        // 准备坐标文本
        std::string text = "Coord: (" + std::to_string(params.mousePoint.x) + 
                           ", " + std::to_string(params.mousePoint.y) + ")";

        // 将坐标画在图像上
        // 参数：图像, 文字, 位置, 字体, 大小, 颜色, 厚度
        cv::putText(displayFrame, text, cv::Point(20, 40), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        // 画一个圆圈标记鼠标位置
        cv::circle(displayFrame, params.mousePoint, 3, cv::Scalar(0, 0, 255), -1);

        // 显示图像
        cv::imshow(winName, displayFrame);

        // 按下 'ESC' 键退出
        auto key = cv::waitKey(1);
        if (key == 27) break;

        if (key == ' ') 
        {


            auto p =getCameraCoordinates(params.mousePoint,camera_matrix,distortion_coeffs, depth);
            
            double pp[3] = {p.x,p.y,p.z};
            cv::Mat_<double> P(3,1,pp);
            std::cout << p<<"\n";
            cv::Mat gun = R*P + T;
            gun = gun/cv::norm(gun);

            std::cout<<"枪管方向的单位向量：\n"<<gun<<"\n";
        }


    }

    cv::destroyAllWindows();
    return 0;


}
