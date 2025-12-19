#include "../../rm-main/include/HikCamera.hpp"
#include "../../rm-main/include/RTSerial.hpp"
#include "../../rm-main/include/fastqueue.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <array>
#include <thread>
#include <vector>


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


io::HikCamera Hik(0.5,17);
io::RTSerial<Packet> ser(20);
static FastQueue<FrameData> Frames(10);


Test test;
int main() {

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
        
        cv::waitKey(1);

        // std::cout<<"quat: "<<frame.quat.w<<" "<<frame.quat.x<<" "<<frame.quat.y<<" "<<frame.quat.z<<"\n";

        test.count(std::chrono::steady_clock::now() - start);
        start = std::chrono::steady_clock::now();

        if(test.num%200 == 0 && test.num != 0) {test.show();test.clear();std::cout<<frame.quat.toRotMat3x3()<<"\n";}
    }
    
    
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

