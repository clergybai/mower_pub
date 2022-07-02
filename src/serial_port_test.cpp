#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;  
    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(9600);
    sp.setTimeout(to);
 
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(500);
    while(ros::ok())
    {
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            n = sp.read(buffer, n);
            for(int i=0; i<n; i++)
            {
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
            sp.write(buffer, n);
        }
        loop_rate.sleep();
    }
    sp.close();
 
    return 0;
}