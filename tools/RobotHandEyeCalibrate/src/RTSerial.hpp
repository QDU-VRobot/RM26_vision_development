#ifndef INCLUDE_RTSERIAL_HPP
#define INCLUDE_RTSERIAL_HPP

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>

#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>
#include <iostream>
#include <cstring>
// File control definitions
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace io {
/**
 * number of serial data bits
 */
enum SerialDataBits {
    SERIAL_DATABITS_5, /**< 5 databits */
    SERIAL_DATABITS_6, /**< 6 databits */
    SERIAL_DATABITS_7, /**< 7 databits */
    SERIAL_DATABITS_8,  /**< 8 databits */
    SERIAL_DATABITS_16,  /**< 16 databits */
};

/**
 * number of serial stop bits
 */
enum SerialStopBits {
    SERIAL_STOPBITS_1, /**< 1 stop bit */
    SERIAL_STOPBITS_1_5, /**< 1.5 stop bits */
    SERIAL_STOPBITS_2, /**< 2 stop bits */
};

/**
 * type of serial parity bits
 */
enum SerialParity {
    SERIAL_PARITY_NONE, /**< no parity bit */
    SERIAL_PARITY_EVEN, /**< even parity bit */
    SERIAL_PARITY_ODD, /**< odd parity bit */
    SERIAL_PARITY_MARK, /**< mark parity */
    SERIAL_PARITY_SPACE /**< space bit */
};

/*! 
 * @class RTSerial
 * @brief 用于串口通信的类，支持自动接收线程和环形缓冲区。
 * @tparam Packet 数据包的结构体类型
 */

template<typename Packet>
class RTSerial
{
public:
    using timePoint = std::chrono::steady_clock::time_point;

    RTSerial    (size_t buffSize = 3, uint8_t head = 0xA5);

    // Destructor
    ~RTSerial   ();

    /*!
     * @brief 打开并配置串口设备
     * @param Device 设备路径 (如 /dev/ttyUSB0)
     * @param Bauds 波特率
     * @param Databits 数据位
     * @param Parity 校验位
     * @param Stopbits 停止位
     * @return int 1:成功, 负数:各种错误码
     */
    int openDevice(const char *Device, const unsigned int Bauds,
                    SerialDataBits Databits = SERIAL_DATABITS_8,
                    SerialParity Parity = SERIAL_PARITY_NONE,
                    SerialStopBits Stopbits = SERIAL_STOPBITS_1);

    // Check device opening state
    bool isDeviceOpen();

    /*!
    @brief Close the connection with the current device
    */
    void closeDevice();

    /*!
     * @brief 开启后台数据接收线程
     * @details 该线程会不断读取串口数据，进行包头校验和CRC校验（如果设置了），并将数据存入环形缓冲区。
     * @param interval_us 读取间隔/延迟阈值（微秒），用于控制读取频率或判断数据新鲜度
     * @param timeOut_ms 读取单个包体剩余数据的超时时间（毫秒）
     * 注意：该实现倾向于获取最新数据，如果缓冲区有旧数据可能会被清除。
     */
    void startReceive(const unsigned int interval_us = 2000,const float timeOut_ms = 1.5);

    /*!
     * @brief 设置CRC校验回调函数
     * @param fuc 一个返回 bool 的函数对象，参数为 Packet 引用。如果校验失败，数据包将被丢弃。
     */
    void setCheckfuc(const std::function<bool(const Packet&)>& fuc); 


    // _____________________________________
    // ::: Read/Write operation on bytes :::

    /*!
     * @brief 从缓冲区获取完整的 Packet 数据包
     * @param data [输出] 存储读取到的数据包
     * @param dataTime [输出] 该数据包接收时的时间点
     * @return true 成功读取
     * @return false 缓冲区为空，无新数据
     */
    bool readPacket(Packet &data, timePoint &dataTime);


    /*!
     @brief read data from databuffer
     @param data : struct that save data,start from Packet second byte,size of read = sizeof( @param data );
     @param dataTime : time that data recived
     @return false : empty is databuffer
     @return ture : read data success
    */
    template<typename Data>
    bool readData(Data &data,timePoint &dataTime);


    bool   writeBytes(const void *Buffer, const size_t NbBytes, ssize_t &NbBytesWritten);
    bool   writeBytes  (const void *Buffer, const size_t NbBytes);

    // Read an array of byte (with timeout)
    ssize_t  readBytes   (void *buffer,const size_t maxNbBytes,const float timeOut_ms = 1);


    // _________________________
    // ::: Special operation :::


    // Empty the received buffer
    char    flushReceiver();

    // Return the number of bytes in the received buffer
    int     available();


private:
    int             fd;

    const size_t    buffSize;

    const ssize_t   dataSize_bytes = static_cast<ssize_t>( sizeof(Packet) );
    const uint8_t   header;
    
    struct buffdata
    {
        Packet data;
        timePoint time;
    };
    std::unique_ptr<buffdata[]>   buffer;
    std::atomic<size_t>       writeIndex{0};
    std::atomic<size_t>       readIndex{0};

    std::function<bool(const Packet&)> check_fuc = nullptr;
    std::thread dataRecThread;
    std::atomic<bool> RecRuning {false};
};


/*!
 \file    serialib.cpp
 \brief   Source file of the class serialib. This class is used for communication over a serial device.
 \author  Philippe Lucidarme (University of Angers)
 \version 2.0
 \date    december the 27th of 2019

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


This is a licence-free software, it can be used by anyone who try to build a better world.
 */



//_____________________________________
// ::: Constructors and destructors :::


/*!
    Constructor of the class RTSerial.
    @brief buffSize ： 
*/
template<typename Packet>
RTSerial<Packet>::RTSerial(size_t buffSize,uint8_t head):buffSize( buffSize + 1 ), fd(-1),header(head)
{
    this->buffer = std::make_unique<buffdata[]>(this->buffSize);
}


/*!
    \brief      Destructor of the class Serial. It close the connection
*/
// Class desctructor
template<typename Packet>
RTSerial<Packet>::~RTSerial()
{
    closeDevice();
}



//_________________________________________
// ::: Configuration and initialization :::



/*!
     \brief Open the serial port
     \param Device : Port name (COM1, COM2, ... for Windows ) or (/dev/ttyS0, /dev/ttyACM0, /dev/ttyUSB0 ... for linux)
     \param Bauds : Baud rate of the serial port.

                \n Supported baud rate for Windows :
                        - 110
                        - 300
                        - 600
                        - 1200
                        - 2400
                        - 4800
                        - 9600
                        - 14400
                        - 19200
                        - 38400
                        - 56000
                        - 57600
                        - 115200
                        - 128000
                        - 256000

               \n Supported baud rate for Linux :\n
                        - 110
                        - 300
                        - 600
                        - 1200
                        - 2400
                        - 4800
                        - 9600
                        - 19200
                        - 38400
                        - 57600
                        - 115200

               \n Optionally supported baud rates, depending on Linux kernel:\n
                        - 230400
                        - 460800
                        - 500000
                        - 576000
                        - 921600
                        - 1000000
                        - 1152000
                        - 1500000
                        - 2000000
                        - 2500000
                        - 3000000
                        - 3500000
                        - 4000000

     \param Databits : Number of data bits in one UART transmission.

            \n Supported values: \n
                - SERIAL_DATABITS_5 (5)
                - SERIAL_DATABITS_6 (6)
                - SERIAL_DATABITS_7 (7)
                - SERIAL_DATABITS_8 (8)
                - SERIAL_DATABITS_16 (16) (not supported on Unix)

     \param Parity: Parity type

            \n Supported values: \n
                - SERIAL_PARITY_NONE (N)
                - SERIAL_PARITY_EVEN (E)
                - SERIAL_PARITY_ODD (O)
                - SERIAL_PARITY_MARK (MARK) (not supported on Unix)
                - SERIAL_PARITY_SPACE (SPACE) (not supported on Unix)
    \param Stopbit: Number of stop bits

            \n Supported values:
                - SERIAL_STOPBITS_1 (1)
                - SERIAL_STOPBITS_1_5 (1.5) (not supported on Unix)
                - SERIAL_STOPBITS_2 (2)

     \return 1 success
     \return -1 device not found
     \return -2 error while opening the device
     \return -3 error while getting port parameters
     \return -4 Speed (Bauds) not recognized
     \return -5 error while writing port parameters
     \return -6 error while writing timeout parameters
     \return -7 Databits not recognized
     \return -8 Stopbits not recognized
     \return -9 Parity not recognized
*/
template<typename Packet>
int RTSerial<Packet>::openDevice(const char *Device, const unsigned int Bauds,
                          SerialDataBits Databits,
                          SerialParity Parity,
                          SerialStopBits Stopbits) {


    // Structure with the device's options
    struct termios options;


    // Open device
    fd = open(Device, O_RDWR | O_NOCTTY);
    // If the device is not open, return -2
    if (fd == -1) return -2;

    // Get the current options of the port
    tcgetattr(fd, &options);
    // 将串口设置为原始数据模式
    cfmakeraw(&options);

    // Prepare speed (Bauds)
    speed_t         Speed;
    switch (Bauds)
    {
    case 110  :     Speed=B110; break;
    case 300  :     Speed=B300; break;
    case 600  :     Speed=B600; break;
    case 1200 :     Speed=B1200; break;
    case 2400 :     Speed=B2400; break;
    case 4800 :     Speed=B4800; break;
    case 9600 :     Speed=B9600; break;
    case 19200 :    Speed=B19200; break;
    case 38400 :    Speed=B38400; break;
    case 57600 :    Speed=B57600; break;
    case 115200 :   Speed=B115200; break;
#if defined (B230400)
    case 230400 :   Speed=B230400; break;
#endif
#if defined (B460800)
    case 460800 :   Speed=B460800; break;
#endif
#if defined (B500000)
    case 500000 :   Speed=B500000; break;
#endif
#if defined (B576000)
    case 576000 :   Speed=B576000; break;
#endif
#if defined (B921600)
    case 921600 :   Speed=B921600; break;
#endif
#if defined (B1000000)
    case 1000000 :   Speed=B1000000; break;
#endif
#if defined (B1152000)
    case 1152000 :   Speed=B1152000; break;
#endif
#if defined (B1500000)
    case 1500000 :   Speed=B1500000; break;
#endif
#if defined (B2000000)
    case 2000000 :   Speed=B2000000; break;
#endif
#if defined (B2500000)
    case 2500000 :   Speed=B2500000; break;
#endif
#if defined (B3000000)
    case 3000000 :   Speed=B3000000; break;
#endif
#if defined (B3500000)
    case 3500000 :   Speed=B3500000; break;
#endif
#if defined (B4000000)
    case 4000000 :   Speed=B4000000; break;
#endif
    default : return -4;
    }
    int databits_flag = 0;
    switch(Databits) {
        case SERIAL_DATABITS_5: databits_flag = CS5; break;
        case SERIAL_DATABITS_6: databits_flag = CS6; break;
        case SERIAL_DATABITS_7: databits_flag = CS7; break;
        case SERIAL_DATABITS_8: databits_flag = CS8; break;
        //16 bits and everything else not supported
        default: return -7;
    }
    int stopbits_flag = 0;
    switch(Stopbits) {
        case SERIAL_STOPBITS_1: stopbits_flag = 0; break;
        case SERIAL_STOPBITS_2: stopbits_flag = CSTOPB; break;
        //1.5 stopbits and everything else not supported
        default: return -8;
    }
    int parity_flag = 0;
    switch(Parity) {
        case SERIAL_PARITY_NONE: parity_flag = 0; break;
        case SERIAL_PARITY_EVEN: parity_flag = PARENB; break;
        case SERIAL_PARITY_ODD: parity_flag = (PARENB | PARODD); break;
        //mark and space parity not supported
        default: return -9;
    }

    // Set the baud rate
    cfsetispeed(&options, Speed);
    cfsetospeed(&options, Speed);
    // Configure the device : data bits, stop bits, parity, no control flow
    // Ignore modem control lines (CLOCAL) and Enable receiver (CREAD)
    options.c_cflag |= ( CLOCAL | CREAD | databits_flag | parity_flag | stopbits_flag);
    options.c_iflag |= ( IGNPAR | IGNBRK );
    // Timer unused
    options.c_cc[VTIME]=0;
    // At least on character before satisfy reading
    options.c_cc[VMIN]=1;
    // Activate the settings
    tcsetattr(fd, TCSANOW, &options);
    // Success
    this->RecRuning = true;
    return 1;

}

template<typename Packet>
bool RTSerial<Packet>::isDeviceOpen()
{
    return fd >= 0;
}

/*!
    @brief Close the connection with the current device
*/
template<typename Packet>
void RTSerial<Packet>::closeDevice()
{
    RecRuning = false;
    
    if (this->dataRecThread.joinable())
    {
        this->dataRecThread.join();
    }

    close (fd);
    fd = -1;
}



/*!
     @brief Write an array of data on the current serial port
     @param Buffer : array of bytes to send on the port
     @param NbBytes : number of byte to send
     @param NbBytesWritten : read success size
     @return true success
     @return false error while writting data
*/
template<typename Packet>
bool RTSerial<Packet>::writeBytes(const void *Buffer, const size_t NbBytes, ssize_t &NbBytesWritten)
{
    // Write data
    NbBytesWritten = write (fd,Buffer,NbBytes);
    if (NbBytesWritten !=(ssize_t)NbBytes) return false;
    // Write operation successfull
    return true;
}

/*!
     @brief Write an array of data on the current serial port
     @param Buffer : array of bytes to send on the port
     @param NbBytes : number of byte to send
     @return 1 success
     @return -1 error while writting data
*/
template<typename Packet>
bool RTSerial<Packet>::writeBytes(const void *Buffer, const size_t NbBytes)
{
    ssize_t NbBytesWritten;
    return writeBytes(Buffer, NbBytes, NbBytesWritten);
}


template <typename Packet>
bool RTSerial<Packet>::readPacket(Packet &data, timePoint &dataTime)
{
    // Check if buffer is empty return
    if( readIndex == writeIndex ) return false;

    // Retrieve data and timestamp
    data = buffer[readIndex].data;
    dataTime = buffer[readIndex].time;

    // Update read index
    readIndex = (readIndex + 1) % buffSize;

    return true;
}


/*!
     @brief read data from databuffer
     @param data : struct that save data,start from Packet second byte,size of read = sizeof( @param data );
     @param dataTime : time that data recived
     @return false : empty is databuffer
     @return ture : read data success
*/
template <typename Packet>
template<typename Data>
bool RTSerial<Packet>::readData(Data &data,timePoint &dataTime)
{
    if( sizeof(Data) > (this->dataSize_bytes-1) )
    {
        std::cerr<<"readData function of RTSerial,that param data size bigger than packet"<<std::endl;
        return false;
    }

    // Check if buffer is empty return
    if( readIndex == writeIndex ) return false;
    
    void* srcPtr = &buffer[readIndex].data;
    srcPtr = (void*)( (uint8_t*)srcPtr + 1 );


    memcpy(&data, srcPtr, sizeof(Data));
    dataTime = buffer[readIndex].time;

    this->readIndex = (readIndex+1)%this->buffSize;
    return true;
}

template<typename Packet>
void RTSerial<Packet>::startReceive(const unsigned int interval_us,const float timeOut_ms)
{
    auto Receive = [this,interval_us,timeOut_ms]() -> void
    {
        std::cout<<"Thread of data Receive is running"<<"\n";

        while (this->RecRuning && isDeviceOpen()) {

            //清空缓冲区，保证读取到的包头都是最新的数据
            if( this->available() > 0 ) this->flushReceiver();

            //判断缓冲区是否已经满,满了就跳过
            if( (writeIndex+1)%this->buffSize == readIndex )
            {
                std::cerr<<"databuffer full"<<std::endl;
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            } 

            void *P = &buffer[writeIndex].data;
            
            //读取包头
            timePoint dataTime = std::chrono::steady_clock::now();
            ssize_t Ret = read(fd,P,sizeof(Packet));

            if (Ret < 1) continue;
            
            if( ( (uint8_t*)P )[0] != this->header ) continue;//如果不是包头跳过

            auto readtime = std::chrono::steady_clock::now();
            if((readtime-dataTime)<std::chrono::microseconds(interval_us)) continue;//不是最新的消息跳过

            //成功读取到正确的包头，读取整个包
            ssize_t Ret_d;
            if( Ret < dataSize_bytes )
            {
                uint8_t *Ptr = (uint8_t*)P + Ret;
                Ret_d = this->readBytes(Ptr, static_cast<size_t>(this->dataSize_bytes - Ret), timeOut_ms);
            }
                

            //如果读取的包长度不够直接跳过
            if( (Ret + Ret_d ) < (this->dataSize_bytes) ) continue;


            //包正常如果有crc函数调用
            if( this->check_fuc ) { if( !this->check_fuc( buffer[writeIndex].data ) ) continue; }

            //数据正常
            buffer[writeIndex].time = readtime;
            writeIndex = ( (writeIndex+1) %this->buffSize );
        }
        std::cout<<"Thread of data Receive is stopped"<<std::endl;
    };

    this->dataRecThread = std::thread(Receive);
}


/*!
     \brief Read an array of bytes from the serial device (with timeout)
     \param buffer : array of bytes read from the serial device
     \param maxNbBytes : maximum allowed number of bytes read
     \param timeOut_ms : delay of timeout before giving up the reading
     \param sleepDuration_us : delay of CPU relaxing in microseconds (Linux only)
            In the reading loop, a sleep can be performed after each reading
            This allows CPU to perform other tasks
     \return >=0 return the number of bytes read before timeout or
                requested data is completed
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
  */
template<typename Packet>
ssize_t RTSerial<Packet>::readBytes (void *buffer,const size_t maxNbBytes,const float timeOut_ms)
{
    auto timeStart = std::chrono::steady_clock::now();

    if(timeOut_ms<0) return -3;

    auto timeOut = std::chrono::microseconds( (long long)(timeOut_ms*1000) );
    int nBytes = 0;

    ioctl(fd, FIONREAD, &nBytes);

    if( nBytes >= (ssize_t)maxNbBytes )
    {
        // Try to read a byte on the device
        ssize_t Ret = read(fd,buffer,maxNbBytes);

        // Error while reading
        if (Ret==-1) return -2;

        // One or several byte(s) has been read on the device
        return Ret;
    }

    // While Timeout is not reached
    size_t NbByteRead=0;
    while (true)
    {
        // 1. 准备 fd_set
        fd_set read_set;
        FD_ZERO(&read_set);       // A. 清空集合
        FD_SET(fd, &read_set);    // B. 将你的串口 fd 加入集合

        //判断是否超时
        auto waitTime = std::chrono::duration_cast<std::chrono::microseconds> (std::chrono::steady_clock::now()-timeStart) ;
        
        if(waitTime>=timeOut) break;
        
        // 计算剩余的微秒总数
        auto remainingTime = timeOut - waitTime;
        long long remaining_us = remainingTime.count();

        // 2. 准备超时时间 
        struct timeval timeout;
        timeout.tv_sec = remaining_us / 1000000;
        timeout.tv_usec = remaining_us % 1000000; // 500,000 微秒 = 500 毫秒

        // 3. 计算 nfds
        // 因为我们只监视一个 fd，所以 nfds 就是它加 1
        int nfds = fd + 1;

        // 4. 调用 select()，线程将在这里阻塞
        int result = select(nfds, &read_set, NULL, NULL, &timeout);

        // 5. 分析结果
        if (result == -1) 
        {
            // 发生错误
            perror("select() error");
            return -1;
        } 

        else if (result == 0) { break; }

        //没有超时获得数据
        uint8_t* Ptr = (uint8_t*)buffer + NbByteRead;
        ssize_t Ret = read(fd,Ptr,maxNbBytes - NbByteRead);
        
        // Error while reading
        if (Ret==-1) return -2;

        // One or several byte(s) has been read on the device
        if (Ret>0)
        {
            // Increase the number of read bytes
            NbByteRead+=Ret;
            // Success : bytes has been read
            if (NbByteRead>=maxNbBytes)
                return NbByteRead;
        }
    }
    //如果超时退出循环体，防止数据在调用select函数之间到达,导致有数据到达但是没检查到就超时，检查一下内核缓冲区数据是否真的没到达
    ioctl(fd, FIONREAD, &nBytes);
    //如果缓冲区域数据不足，直接返回读取到的数据
    if( nBytes < (ssize_t)(maxNbBytes-NbByteRead) )  return NbByteRead;
    
    //如果缓冲区域数据足够，合并再返回
    ssize_t Ret = read(fd, (uint8_t*)buffer + NbByteRead, maxNbBytes - NbByteRead);
    if(Ret >= 0) return (NbByteRead+=Ret);

    std::cout << "Timeout: No data available within timeOut." << std::endl;
    // Error while reading
    return NbByteRead;
}


//设置crc校验函数
template<typename Packet>
void RTSerial<Packet>::setCheckfuc(const std::function<bool(const Packet&)>& fuc)
{   
    this->check_fuc = fuc;
}



// _________________________
// ::: Special operation :::



/*!
    \brief Empty receiver buffer
    Note that when using serial over USB on Unix systems, a delay of 20ms may be necessary before calling the flushReceiver function
    \return If the function succeeds, the return value is nonzero.
            If the function fails, the return value is zero.
*/
template<typename Packet>
char RTSerial<Packet>::flushReceiver()
{
    // Purge receiver
    tcflush(fd,TCIFLUSH);
    return true;

}



/*!
    \brief  Return the number of bytes in the received buffer (UNIX only)
    \return The number of bytes received by the serial provider but not yet read.
*/
template<typename Packet>
int RTSerial<Packet>::available()
{    

    int nBytes=0;
    // Return number of pending bytes in the receiver
    ioctl(fd, FIONREAD, &nBytes);
    return nBytes;
}



class CRC8
{
 private:
  static const uint8_t INIT = 0xFF;  ///< CRC8 初始值 / CRC8 initial value

 public:
  static inline uint8_t tab_[256];  ///< CRC8 查找表 / CRC8 lookup table
  static inline bool inited_ =
      false;  ///< 查找表是否已初始化 / Whether the lookup table is initialized

  CRC8() {}

  /**
   * @brief 生成 CRC8 查找表 / Generates the CRC8 lookup table
   */
  static void GenerateTable()
  {
    uint8_t crc = 0;

    for (int i = 0; i < 256; i++)
    {
      tab_[i] = i;
    }

    for (int i = 0; i < 256; i++)
    {
      for (int j = 7; j >= 0; j--)
      {
        crc = tab_[i] & 0x01;

        if (crc)
        {
          tab_[i] = tab_[i] >> 1;
          tab_[i] ^= 0x8c;
        }
        else
        {
          tab_[i] = tab_[i] >> 1;
        }
      }
    }
    inited_ = true;
  }

  /**
   * @brief 计算数据的 CRC8 校验码 / Computes the CRC8 checksum for the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 计算得到的 CRC8 值 / Computed CRC8 value
   */
  static uint8_t Calculate(const void *raw, size_t len)
  {
    const uint8_t *buf = reinterpret_cast<const uint8_t *>(raw);
    if (!inited_)
    {
      GenerateTable();
    }

    uint8_t crc = INIT;

    while (len-- > 0)
    {
      crc = tab_[(crc ^ *buf++) & 0xff];
    }

    return crc;
  }

  /**
   * @brief 验证数据的 CRC8 校验码 / Verifies the CRC8 checksum of the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 校验成功返回 `true`，否则返回 `false` /
   *         Returns `true` if the checksum is valid, otherwise returns `false`
   */
  static bool Verify(const void *raw, size_t len)
  {
    const uint8_t *buf = reinterpret_cast<const uint8_t *>(raw);
    if (!inited_)
    {
      GenerateTable();
    }

    if (len < 2)
    {
      return false;
    }
    uint8_t expected = Calculate(buf, len - sizeof(uint8_t));
    return expected == buf[len - sizeof(uint8_t)];
  }
  template<typename Packet>

  static bool Check(const Packet& data)
  {
    const void* ptr = &data;
    return Verify(ptr, sizeof(Packet));
  }
};



}//namespace io

#endif // serialib_H