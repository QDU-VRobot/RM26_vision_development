#include "HikCamera.hpp"
#include "fastqueue.hpp"
#include "RTSerial.hpp"
#include <opencv2/core/quaternion.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/matx.hpp>
#include <string>
#include <iomanip>    // 用于格式化时间字符串
#include <sstream>    // 用于构建字符串
#include <thread>



struct __attribute__((packed)) Packet{

    uint8_t header;       // 0xA5
    uint8_t target_id;    // 0xEA
    uint8_t length;       // 0x1A (26)
    uint8_t cmd_id;       // 0x35
    uint8_t head_chk;     // 0xA6
    uint32_t timestamp;   // 0x7A100000 (Little Endian) or ID
    float q0;             // x
    float q1;             // y
    float q2;             // z
    float q3;             // w
    uint8_t checksum;     // 校验和
} ;

struct FrameData
{
    cv::Mat image;
    cv::Quatf quat;
    std::chrono::steady_clock::time_point time;

    FrameData(const cv::Mat image, const cv::Quatf& quat,
              const std::chrono::steady_clock::time_point& time)
        : image(image), quat(quat), time(time) {}
    FrameData(){}
};

//性能测试工具
struct Test
{
    int num = 0;
    std::chrono::nanoseconds total{0};

    void count(const std::chrono::nanoseconds& time);
    void clear();
    void show();
};

//IMU与图像配对线程
void IMUAndImageMatchThread(io::HikCamera& Hik, io::RTSerial<Packet>& ser,FastQueue<FrameData>& Frames);


io::HikCamera Hik(2,17);
io::RTSerial<Packet> ser(20);
static FastQueue<FrameData> Frames(10);


Test test;


// 函数：生成一个基于当前时间的唯一文件名
static int Num=0;

std::string generate_filename(int &Num) {
    
    std::string ss;
    ss ="image_"+std::to_string(Num++)+"_";
    return ss;
}

int main() {
    // 1. 定义数据保存目录
    std::string output_dir = "../Data/images";
    std::string config_path = "../Data/Calibration_R_T.yaml";

    //加载存储数据的YAML文件
    cv::FileStorage fs;
    if (!fs.open(config_path, cv::FileStorage::WRITE)) {
        std::cerr << "Error: Failed to open YAML file: " << config_path << std::endl;
        return -1;  // 失败时返回
    }


    std::cout << "Successfully opened " << config_path << std::endl;
    std::cout << "------------------------------------------" << std::endl;





    //1.0初始化串口
    std::cout<<sizeof(Packet)<<std::endl;

    std::function<bool(const Packet&)> check_fuc = io::CRC8::Check<Packet>;
    ser.setCheckfuc(check_fuc);
    int ret = ser.openDevice("/dev/ttyACM0", 460800);
    
    if(ret == 1)
        std::cout<<"serial open ok"<<"\n";
    else
        std::cerr<<"serial open err: "<<ret<<"\n";
    
    ser.startReceive(100);


    //2.0初始化相机
    Hik.continueCap(5);

    //3.0创建数据配对线程，并将数据发布到Frames环形队列
    std::thread match_thread = std::thread(IMUAndImageMatchThread, std::ref(Hik), std::ref(ser), std::ref(Frames));
    
    cv::namedWindow("frame");

    auto start = std::chrono::steady_clock::now();
    while(true)
    {
        FrameData frame;
        bool haveData = Frames.pop(frame);
        
        if(!haveData) continue;
        
        //如果不是最新照片直接跳过直到拿到最新照片
        if(!Frames.empty()) continue;

        cv::imshow("frame", frame.image);
        
        // 等待按键事件 (等待1毫秒)
        // 这个延时对于显示视频至关重要，否则窗口会无响应
        int key = cv::waitKey(1);

        // 9. 处理按键
        if (key == ' ') { // 空格键的ASCII码是32
            // 生成文件名并拼接完整路径
            std::string filename = generate_filename(Num);
            std::string filepath = output_dir + "/" + filename;

            // 保存当前帧为PNG图片
            bool saved = cv::imwrite(filepath+".png", frame.image);
            cv::Mat R_grip_to_world(frame.quat.toRotMat3x3());
            cv::Mat R_world_to_grip = R_grip_to_world.t();

            fs << filename << R_world_to_grip;
            std::cout<<R_world_to_grip<<"\n";

            
            if (saved) {
                std::cout << "图片已保存: " << filepath << std::endl;
            } else {
                std::cerr << "错误: 无法保存图片到 " << filepath << std::endl;
            }
        } else if (key == 27) { // ESC键的ASCII码是27
            std::cout << "正在退出程序..." << std::endl;
            break; // 退出循环
        }


        test.count(std::chrono::steady_clock::now() - start);
        start = std::chrono::steady_clock::now();

        if(test.num%200 == 0 && test.num != 0) {test.show();test.clear();}
    }

    fs.release();// 关闭文件
    cv::destroyAllWindows();
    match_thread.detach();
    return 0;
}


//IMU与图像配对线程
void IMUAndImageMatchThread(io::HikCamera& Hik, io::RTSerial<Packet>& ser,FastQueue<FrameData>& Frames)
{
    while (true) {

        // 读取相机数据
        io::HikCamera::ImageData HikData;

        Hik.read(HikData);
        if(HikData.image.empty()) continue;

        // 读取串口数据
        std::chrono::steady_clock::time_point time ;
        Packet IMU;
        while( true )
        {
            bool ret = ser.readPacket(IMU, time);
            if( !ret ) break;

            const auto& t = ((double)(HikData.time - time).count()) * 1e-6;

            //配对超时

            //串口数据比相机数据早8ms以上
            if( t > 8 ) continue;

            //串口数据比相机数据早5ms以下
            if( t < 5 ) break;

            //配对成功
            cv::Quatf quat( IMU.q3, IMU.q0, IMU.q1, IMU.q2 );
            FrameData frame(HikData.image, quat, HikData.time);

            Frames.push(frame);
            break;
        }

    }
}


//性能测试工具
void Test::count(const std::chrono::nanoseconds& time)
{
    this->num++;
    total += time;
}

void Test::clear()
{
    this->num = 0;
    this->total = std::chrono::nanoseconds(0);        
}

void Test::show()
{
    std::cout<< this->num / ( ( (double)this->total.count() ) * 1e-9 )<< "Hz\n" ;
}

