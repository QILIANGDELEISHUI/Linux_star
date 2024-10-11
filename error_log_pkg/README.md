
rosrun Error_Log_pkg Error_Log_node 启动节点

// 日志文件创建路径
#define LOG_CRRETION_PATH "/home/star/catkin_ws/src/error_log_pkg/launch/"
// 日志文件上传路径
#define LOG_UPLOAD_PATH "/ros_log/"
// FTP服务器IP
#define FTP_SERVER_IP "192.168.31.193"
// FTP服务器用户名和密码
#define FTP_USERPWD "star:star"

在日志文件创建路径下生成文件名为当前时间的.log文件。
例如：2024-04-25_17-27-10.log。
同时命令行显示：[ INFO] [1714037908.558238176]: Log file '2024-04-25_17-38-28.log' created. 表示log创建成功
反之Failed to create log file  '2024-04-25_17-38-28.log'。表示文件创建失败
文件创建成功后，会在2024-04-25_17-27-10.log文件中输出带时间戳的信息。
例如：[2024-04-25 17:27:14]dai fei；
只有当接收到的消息不为空时才会把消息写入.log文件中
目前，消息内容和消息时间都重复，消息可以写入.log文件