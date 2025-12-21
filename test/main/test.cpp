#include "../../rm-main/include/HikCamera.hpp"
#include "../../rm-main/include/RTSerial.hpp"
#include "../../rm-main/include/fastqueue.hpp"
#include "../../rm-main/include/Detector.hpp"
#include "../../rm-main/include/Solver.hpp"
#include "../../rm-main/include/Shooter.hpp"
#include "../../rm-main/include/Tracker.hpp"
#include "../../rm-main/include/ShootTable.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
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
struct __attribute__((packed)) ShootPosi{

    uint8_t header = 0xA5;       // 0xA5
    uint8_t id_1 = 0xCB;    // 0xEA
    uint8_t id_2 = 0x86 ;       // 0x1A (26)
    uint8_t id_3 = 0x09 ;       // 0x35
    uint8_t id_4 = 0x6F ;;     // 0xA6
    uint8_t len_1 =  0x00 ;   // 0x7A100000 (Little Endian) or ID
    uint8_t len_2 = 0x00 ;
    uint8_t len_3 = 0x0C;
    uint8_t crc_head = 0x55;
    
    float row; 
    float pitch;            // x
    float yaw;             // y
            
    uint8_t checksum;     // 校验和
};

struct __attribute__((packed)) ShootFire{

    uint8_t header = 0xA5;       // 0xA5
    uint8_t id_1 = 0x6B;    // 0xEA
    uint8_t id_2 = 0xAD ;       // 0x1A (26)
    uint8_t id_3 = 0x91 ;       // 0x35
    uint8_t id_4 = 0x02 ;;     // 0xA6
    uint8_t len_1 =  0x00 ;   // 0x7A100000 (Little Endian) or ID
    uint8_t len_2 = 0x00 ;
    uint8_t len_3 = 0x01;
    uint8_t crc_head = 0x1A;

    uint8_t fire;             // z

    uint8_t checksum;     // 校验和
};


struct FrameData
{
    cv::Mat image;
    cv::Quatd quat;
    std::chrono::steady_clock::time_point time;

    FrameData(const cv::Mat image, const cv::Quatd& quat,
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


io::HikCamera Hik(1,17);
io::RTSerial<Packet> ser(20);
static FastQueue<FrameData> Frames(10);

Detector detect(Light::Color::Red,0.5,"../../../rm-main/model/mobilenet_v3_112_rgb.onnx");
Solver Sov("../../../config/Solver_config.yaml");
Tracker track;

ShootTable::TableConfig tableconfig(10,0,2,-1,0.01,"/home/king/AUTO-Aming-system/config/infantry_10_table.bin");
Shooter shoot(cv::Point3d(-0.9996123276310385,0.02082249458349189, -0.01848291555403893),tableconfig);

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



        //识别
        detect.rgb_img = frame.image;
        auto binary_img = detect.preprocessImage(frame.image); //预处理图像

        auto lights = detect.FindLight(binary_img); //寻找灯条

        auto possible_armors = detect.FindArmor(lights); //寻找装甲板
        
        if(possible_armors.empty()) {continue;}
        //将装甲板转换为vector
        std::vector<Armor> armors;
        armors.reserve(possible_armors.size());
        for(const auto& armor : possible_armors)
        {
            armors.push_back(armor);
        }

        detect.ArmorShow(frame.image, armors);
        // cv::imshow("frame", frame.image);
        
        // cv::waitKey(1);
        armors[0].confidence = 1.0;
        armors[0].type = Armor::Type::guard;
        //解算装甲板位置
        auto armors_posi = Sov(armors);

        // if(test.num%100 == 0 && test.num != 0)
        // {
        //     std::cout<<armors_posi[0].posi<<"\n";
        // }
        Sov.ConverToWorld(armors_posi,frame.quat);


        // if(test.num%100 == 0 && test.num != 0)
        // {
        //     std::cout<<armors_posi[0].posi/10<<"\n";
        // }
        // std::cout<<"quat: "<<frame.quat.w<<" "<<frame.quat.x<<" "<<frame.quat.y<<" "<<frame.quat.z<<"\n";

        test.count(std::chrono::steady_clock::now() - start);
        start = std::chrono::steady_clock::now();

        if(test.num%200 == 0 && test.num != 0) {test.show();
            test.clear();}


        //打弹


        // std::this_thread::sleep_for(std::chrono::nanoseconds(100000000));

        //traker:

        Eigen::Matrix<double, 3, 1> posi;
        posi << armors_posi[0].posi.x, armors_posi[0].posi.y, armors_posi[0].posi.z;

        auto ans = track(posi,0.004);
        // std::cout<< "Filtered Position: " << ans.transpose() << std::endl;

        float dt = shoot.FlyTime(armors_posi[0].posi/1000); 

        cv::Point3d predict_posi;
        predict_posi.x = (ans(0,0) + dt * ans(3,0)) ;
        predict_posi.y = (ans(1,0) + dt * ans(4,0)) ;
        predict_posi.z = (ans(2,0) + dt * ans(5,0)) ;

        predict_posi = predict_posi/1000;//单位换算到m

        std::array<double, 2> Pitch_and_Yaw = shoot(predict_posi);
        // std::cout<<Pitch_and_Yaw[0]<<" "<<Pitch_and_Yaw[1]<<"\n";
        ShootPosi sed1;
        sed1.row =0 ;
        sed1.pitch = (float)Pitch_and_Yaw[0];
        // std::cout<<sed1.pitch<<"\n";
        sed1.yaw = (float)Pitch_and_Yaw[1];
        // std::cout<<sed1.pitch<<"  "<<sed1.yaw<<"\n";
        sed1.checksum = io::CRC8::Calculate(&sed1, sizeof(sed1)-1);

        ShootFire sed2;
        sed2.fire = 1;
        sed2.checksum = io::CRC8::Calculate(&sed2, sizeof(sed2)-1);

        ser.writeBytes(&sed1,sizeof(sed1));
        ser.writeBytes(&sed2,sizeof(sed2));
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
            cv::Quatd quat( IMU.q3, IMU.q0, IMU.q1, IMU.q2 );
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

