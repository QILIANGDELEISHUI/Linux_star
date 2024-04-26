#include <ros/ros.h> // 包含ROS头文件
#include <fstream>   // 包含文件流头文件
#include <iostream>  // 包含标准输入输出流头文件
#include <sstream>   // 包含字符串流头文件
#include <ctime>     // 包含时间相关头文件
#include <std_msgs/String.h>

// 全局变量存储回调函数获取到的消息
std::string error_msg;
// 消息订阅回调函数，订阅的消息类型为std_msgs::String
void Error_callback(std_msgs::String msg)
{
    ROS_INFO(msg.data.c_str());
    error_msg = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Error_Log_node"); // 初始化ROS节点
    ros::NodeHandle nh;                      // 创建ROS节点句柄

    // 获取当前系统时间
    std::time_t current_time = std::time(nullptr);             // 获取当前时间的时间戳
    std::tm *time_info = std::localtime(&current_time);        // 将时间戳转换为本地时间
    char buffer[80];                                           // 创建缓冲区
    std::strftime(buffer, 80, "%Y-%m-%d_%H-%M-%S", time_info); // 格式化时间字符串
    std::string log_file_name(buffer);                         // 创建以时间命名的.log文件名
    log_file_name += ".log";                                   // 添加.log后缀

    // 创建.log文件
    std::ofstream log_file(log_file_name); // 创建输出文件流
    // 订阅YHS_CIR02/Error话题，订阅队列的大小为10，有订阅消息时，调用Error_callback回调函数
    ros::Subscriber sub = nh.subscribe("YHS_CIR02/Error", 10, Error_callback);

    if (log_file.is_open())
    {                                                                   // 检查文件是否成功打开
        ROS_INFO_STREAM("Log file '" << log_file_name << "' created."); // ROS日志输出

        // 写入消息到.log文件
        log_file << "Log file created at: " << log_file_name << std::endl; // 写入创建日志文件的消息
        log_file << "Starting log messages:" << std::endl;                 // 写入开始日志消息的标记

        // ros::Rate loop_rate(1); // 创建ROS循环频率对象，每秒一次

        while (ros::ok())
        { // 循环直到ROS节点停止
            // 获取当前时间并生成带时间戳的消息
            std::time_t current_time = std::time(nullptr);                         // 获取当前时间的时间戳
            std::tm *time_info = std::localtime(&current_time);                    // 将时间戳转换为本地时间
            char timestamp_buffer[80];                                             // 创建缓冲区
            std::strftime(timestamp_buffer, 80, "[%Y-%m-%d %H:%M:%S]", time_info); // 格式化时间字符串

            std::stringstream message; // 创建字符串流对象

            //如果error_msg不为空
            if (!error_msg.empty())
            {
                message << timestamp_buffer << error_msg; // 构建带时间戳的日志消
                log_file << message.str() << std::endl; // 将消息写入日志文件
                error_msg.clear();
            }
            ros::spinOnce(); // 处理ROS回调函数
            // loop_rate.sleep(); // 休眠以满足循环频率
        }

        log_file.close(); // 关闭日志文件
    }
    else
    {
        ROS_ERROR_STREAM("Failed to create log file '" << log_file_name << "'."); // 日志文件创建失败
    }

    return 0; // 返回0表示正常退出
}