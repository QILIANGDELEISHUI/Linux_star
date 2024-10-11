#include <ros/ros.h>   // 包含ROS头文件
#include <fstream>     // 包含文件流头文件
#include <iostream>    // 包含标准输入输出流头文件
#include <sstream>     // 包含字符串流头文件
#include <ctime>       // 包含时间相关头文件
#include <curl/curl.h> //包含curl库等头文件
#include <std_msgs/String.h>

// 日志文件创建路径
#define LOG_CRRETION_PATH "/home/star/catkin_ws/src/error_log_pkg/launch/"
// 日志文件上传路径
#define LOG_UPLOAD_PATH "/ros_log/"
// FTP服务器IP
#define FTP_SERVER_IP "192.168.31.193"
// FTP服务器用户名和密码
#define FTP_USERPWD "star:star"

// 全局变量存储回调函数获取到的消息
std::string error_msg;
// 消息订阅回调函数，订阅的消息类型为std_msgs::String
void Error_callback(std_msgs::String msg)
{
    ROS_INFO("%s", msg.data.c_str());
    error_msg = msg.data;
}
// 获取实施时间的字符串
std::string real_time_time_string()
{
    // 获取当前系统时间
    std::time_t current_time = std::time(nullptr);             // 获取当前时间的时间戳
    std::tm *time_info = std::localtime(&current_time);        // 将时间戳转换为本地时间
    char buffer[80];                                           // 创建缓冲区
    // /home/star/catkin_ws/src/error_log_pkg/launch
    std::strftime(buffer, 80, "%Y-%m-%d_%H-%M-%S", time_info); // 格式化时间字符串
    return std::string(buffer);                                // 返回实施时间字符串
}
//ftp上传
CURLcode ftp_upload_files(std::string &ftp_url, std::string &log_file_name)
{
    //CURLE_OBSOLETE20在定义文件中显示已经不再使用
    //此处只是为了初始化res的值，值本身没有含义，可更改。
    CURLcode res = CURLE_OBSOLETE20;

    // 以读取的方式打开要本地上传的文件
    FILE *in_log_file = fopen((LOG_CRRETION_PATH + log_file_name).c_str(), "rb");
    if (in_log_file)
    {
        curl_global_init(CURL_GLOBAL_DEFAULT); // 初始化curl库 初始化所有可能的全局变量
        CURL *curl = curl_easy_init();         // 创建一个CURL对象

        // ROS_INFO("in_log_file.is_open()");
        //  获取文件大小
        fseek(in_log_file, 0, SEEK_END);
        long filesize = ftell(in_log_file);
        rewind(in_log_file);

        // ROS_INFO("ftp_url:%s", ftp_url.c_str());
        //  设置FTP服务器的URL
        curl_easy_setopt(curl, CURLOPT_URL, ftp_url.c_str());
        // 设置FTP服务器的用户名和密码
        curl_easy_setopt(curl, CURLOPT_USERPWD, FTP_USERPWD);
        // 设置上传文件的数据源
        curl_easy_setopt(curl, CURLOPT_READDATA, in_log_file);
        // 设置预期上传的字节数
        curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, (curl_off_t)filesize);
        // 设置上传操作
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
        // 执行FTP上传操作
        CURLcode res = curl_easy_perform(curl);
        ROS_INFO("res=%d", res);
        // 关闭文件
        fclose(in_log_file);
        // ROS_INFO("in_log_file.is_close()");

        // 清理并关闭CURL对象
        curl_easy_cleanup(curl);
        // 清理CURL库
        curl_global_cleanup();
        return res;
    }
    else
    {
        ROS_ERROR("Failed to open file");
        return res;
    }

}
//断点续传
//单个文件的断点续传测试 构造断点续传场景
//发现断点续传，这种方案并不可靠，分开构造curl分别获取服务端文件大小，进行续传处理
CURLcode on_pb_restart_one_clicked(std::string &ftp_url, std::string &log_file_name)
{
    // 断点续传的步骤（上传了一半要接着传）
    //1.获取ftp云服务中指定路径指定文件的数据量
    //2.获取本地文件的数据量
    //3.对比如果相等就退出
    //4.如果云服务器文件的数据量少于本地文件就准备断点续传
    //5.断点续传的核心是打开云服务的指定文件，从末端开始，
    //本地文件则是从云服务器文件的末端数据量开始上传。



    //CURLE_OBSOLETE20在定义文件中显示已经不再使用
    //此处只是为了初始化res的值，值本身没有含义，可更改。
    CURLcode res = CURLE_OBSOLETE20;
    curl_global_init(CURL_GLOBAL_ALL);// 初始化curl库 初始化默认的全局环境设置
    CURL *curl = curl_easy_init();// 创建一个CURL对象

    // 以读取的方式打开要本地上传的文件
    FILE *in_log_file = fopen((LOG_CRRETION_PATH + log_file_name).c_str(), "rb");
    // FILE *hd_src = fopen(ftp_file.toStdString().c_str(), "rb");
    if (!in_log_file)
    {
        // ui->Display_Edit->appendPlainText("打开文件失败:" + ftp_file);
        curl_global_cleanup();
        return res;
    }

    //  设置FTP服务器的URL
    curl_easy_setopt(curl, CURLOPT_URL, ftp_url.c_str());
    // 设置FTP服务器的用户名和密码
    curl_easy_setopt(curl, CURLOPT_USERPWD, FTP_USERPWD);
    // 设置上传操作
    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

    long uploaded_len = 0;
    //设置一个回调函数 getcontentlengthfunc 用于处理响应头部。
    // curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, getcontentlengthfunc);
    //传递给 CURLOPT_HEADERFUNCTION 设置的回调函数的用户数据。在这里，将 &uploaded_len 作为用户数据传递，可以在回调函数中使用这个指针来更新 uploaded_len 的值
    curl_easy_setopt(curl, CURLOPT_HEADERDATA, &uploaded_len);
    // curl_easy_getinfo();

    //CURLE_GOT_NOTHING==获取到空内容
    res = CURLE_GOT_NOTHING;
    for (int i = 0; (i < 3) && (res != CURLE_OK); ++i)
    {
        //只获取响应头信息  而不实际下载响应体
        curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);
        //响应头信息包含在返回的输出中
        //响应头信息包含在返回的输出中
        curl_easy_setopt(curl, CURLOPT_HEADER, 1L);
        //提交之前所有的curl_easy_setopt设置
        //这里是获取远程服务器文件的大小
        res = curl_easy_perform(curl);
        if (res != CURLE_OK)
            continue;
        curl_easy_cleanup(curl); //获取后，先清理，再重新进行必要的设置。

        //重新定义一个空的curl
        curl = curl_easy_init();
        //  设置FTP服务器的URL
        curl_easy_setopt(curl, CURLOPT_URL, ftp_url.c_str());
        // 设置FTP服务器的用户名和密码
        curl_easy_setopt(curl, CURLOPT_USERPWD, FTP_USERPWD);
        // 设置上传操作
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
        //设置一个回调函数 discardfunc 用于处理响应体数据。
        // curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, discardfunc);
        //设置一个回调函数 readfunc 用于提供上传数据。 
        // curl_easy_setopt(curl, CURLOPT_READFUNCTION, readfunc);
        //将 in_log_file 作为 readfunc 回调函数的用户数据传递。通常，in_log_file 是包含要上传数据的缓冲区或结构体。
        curl_easy_setopt(curl, CURLOPT_READDATA, in_log_file);
        //指定 FTP 传输的端口号。
        curl_easy_setopt(curl, CURLOPT_FTPPORT, "-");
        //指定接受连接的超时时间（以毫秒为单位），本例为7000毫秒（7秒）
        curl_easy_setopt(curl, CURLOPT_ACCEPTTIMEOUT_MS, 7000L);
        //自动创建缺失目录 在上传文件时，如果目标目录不存在，libcurl 会自动创建这些目录。
        curl_easy_setopt(curl, CURLOPT_FTP_CREATE_MISSING_DIRS, 1L);
        //启用详细的调试输出。
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        // ui->Display_Edit->appendPlainText("获取到服务器文件大小为：" + QString::number(uploaded_len));
        //下载响应体
        curl_easy_setopt(curl, CURLOPT_NOBODY, 0L);
        //不会将响应头部包含在返回的输出中，只下载响应体。
        curl_easy_setopt(curl, CURLOPT_HEADER, 0L);

        // fseek(hd_src, uploaded_len, SEEK_SET); //把hd_src从开始位置偏移uploaded_len长度
        //如果目标文件已存在，新的数据会追加到文件末尾，而不是覆盖现有文件内容。
        curl_easy_setopt(curl, CURLOPT_APPEND, 1L);
    }

    if (res != CURLE_OK)
    {
        //默认情况下，上传的数据会覆盖目标文件中的现有数据。如果设置为 0L，则不进行追加。
        curl_easy_setopt(curl, CURLOPT_APPEND, 0L);
    }
    //提交之前所有的curl_easy_setopt设置
    //这里是根据服务器的文件大小进行续传
    res = curl_easy_perform(curl);

    // if (res == CURLE_OK)
    //     // ui->Display_Edit->appendPlainText("断点续传文件成功 ！");
    // else
    //     // ui->Display_Edit->appendPlainText("断点续传文件失败 ! " + QString(curl_easy_strerror(res)));

    fclose(in_log_file);
    // 清理并关闭CURL对象
    curl_easy_cleanup(curl);
    // 清理CURL库
    curl_global_cleanup();
    return res; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "error_log_node"); // 初始化ROS节点
    ros::NodeHandle nh;                      // 创建ROS节点句柄
    // 指定日志文件创建路径，名称为实时时间戳+。log
    std::string log_file_name = real_time_time_string() + ".log"; // 创建以时间命名的.log文件名
    // ROS_INFO("log_file_name:%s", log_file_name.sudo dpkg -i yourfile.debc_str());
    // 以写入的方式打开
    std::ofstream out_log_file(LOG_CRRETION_PATH + log_file_name); // 创建.log文件
    // 订阅YHS_CIR02/Error话题，订阅队列的大小为10，有订阅消息时，调用Error_callback回调函数
    ros::Subscriber sub = nh.subscribe("YHS_CIR02/Error", 10, Error_callback);

    // 打开日志文件
    if (out_log_file.is_open())
    { // 检查文件是否成功打开
        // ROS_INFO_STREAM("Log file '" << log_file_name << "' created."); // ROS日志输出

        // 写入消息到.log文件
        out_log_file << "Log file created at: " << log_file_name << std::endl; // 写入创建日志文件的消息
        out_log_file << "Starting log messages:" << std::endl;                 // 写入开始日志消息的标记
        // ros::Rate loop_rate(1); // 创建ROS循环频率对象，每秒一次
        // ros节点停止或消息队列为空
        while (ros::ok() && ros::master::check())
        {                                                     // 循环直到ROS节点停止
            std::string error_time = real_time_time_string(); // 获取实时时间
            std::stringstream message;                        // 创建字符串流对象
            // 如果error_msg不为空
            if (!error_msg.empty())
            {
                message << error_time << error_msg; // 构建带时间戳的日志消息
                ROS_INFO("error_msg");
                out_log_file << message.str() << std::endl; // 将消息写入日志文件
                error_msg.clear();
            }
            ros::spinOnce(); // 处理ROS回调函数
            // loop_rate.sleep(); // 休眠以满足循环频率
        }
        out_log_file.close(); // 关闭日志文件
        // ROS_INFO("out_log_file.close");

        //拼接ftp_url字符串
        std::string ftp_url = "ftp://" + std::string(FTP_SERVER_IP) + std::string(LOG_UPLOAD_PATH) + log_file_name;
        //ftp上传
        CURLcode res = ftp_upload_files(ftp_url, log_file_name);
        ROS_ERROR("curl_easy_perform() failed: %s", curl_easy_strerror(res)); // 输出错误信息

        //断点续传

        // CURLE_FTP_ACCEPT_TIMEOUT: FTP接受超时。
        // CURLE_PARTIAL_FILE: 文件部分传输。
        // CURLE_WRITE_ERROR: 写入错误。
        // CURLE_UPLOAD_FAILED: 上传失败。
        // CURLE_OPERATION_TIMEDOUT: 操作超时。
        // CURLE_FILE_COULDNT_READ_FILE: 无法读取文件。
        // CURLE_ABORTED_BY_CALLBACK: 回调中止。
        // CURLE_BAD_FUNCTION_ARGUMENT: 函数参数错误。
        // CURLE_SEND_ERROR: 发送数据失败。
        // CURLE_FILESIZE_EXCEEDED: 文件大小超过限制
        // CURLE_SEND_FAIL_REWIND: 发送失败，需倒带。
        // CURLE_FTP_BAD_FILE_LIST: 无法解析FTP文件列表
        // if (res != CURLE_OK)
        // {
        //     if (res == CURLE_OBSOLETE20)
        //     {
        //         //Failed to open file
        //     }
        //     else if (res == CURLE_UPLOAD_FAILED)
        //     {
        //         //上传失败
        //         // 上传中断，删除部分上传的文件
        //         delete_partial_upload();

        //         // 重新上传文件
        //         res = curl_easy_perform(curl);
        //         if (res != CURLE_OK)
        //         {
        //             fprintf(stderr, "重新上传失败: %s\n", curl_easy_strerror(res));
        //         }
        //         else
        //         {
        //             printf("重新上传成功\n");
        //         }
        //     }
        //     else if (res == CURLE_UPLOAD_FAILED)
        //     {
        //         //发送数据失败
        //     }
        //     else if (CURLE_SEND_FAIL_REWIND)
        //     {
        //         //发送失败，需倒带
        //     }
        //     else if (res == CURLE_ABORTED_BY_CALLBACK)
        //     {
        //         //回调中止
        //     }
        // }
    }
    else
    {
        ROS_ERROR_STREAM("Failed to create log file '" << log_file_name << "'."); // 日志文件创建失败
    }
    return 0;
}