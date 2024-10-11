# 包简介
python通过sqlite3、pymysql连接sqlite3、mysql数据库
将订阅本地ros话题，通过回调函数写入到sqlite3和云端mysql中
# src目录
config.py 将默认配置字段写入config.db中
log_sql.py为日志系统核心程序 
log_sqlite3.py sqlite3操作函数
log_mysql.py mysql操作函数
send_sql.py sqlite同步到mysql操作函数
pub_cs.py ros话题发布测试
cs.py 测试
# 功能介绍
config.py 将默认配置字段写入config.db中 涉及服务器参数"server"、网络配置"network"，
用户配置"user"（目前时写入默认值，但实际应该设备激活后再写入）、文件保存位置"file"。
config.db中核心字段名称为唯一约束，不允许重复 系统启动时启动顺序为第一优先级
config.db文件保存位置目前是config.py main函数中定义，后续也可更改为文件参数传递或者函数参数传递
log_sql.py进行ros包的初始化，sqlite3和mysql数据库表格的初始化，并从config.py获取必要参数
log_sql.py主程序分别订阅"stop"、"YHS_CIR02/Error"、"lightcon/movestatus"、"battery_pub"、"aikit/wakenUp"话题，
且额外传递话题名topic, 操作的表格名table_name, mysql配置信息config_mysql三个参数。
log_sqlite3.py log_mysql.py send_sql.py三个包中的所有参数都来自log_sql.py中传递(除msg)，其余参数由config.db定义
mysql同步逻辑：mysql掉线后，数据除写入标准log.db中外，还会写入到middle.db中。当mysql重新上线后，将middle.db消息同步到mysql中，同步完成后会删除middle.db中的所有数据(保留表结构)
所有函数都增加了异常抛出,确保不会因为异常中断程序
只要收到消息就进行在线判断。但如果数据接收频率较高,可能会导致数据丢失。
# 使用
start:rosrun error_sql_pkg error_sql_node.py.
# 部分参数介绍
config_mysql:mysql参数配置
config_vsftpd:vsftpd服务器参数配置
SQLITE3_PATH:SQLite3数据库文件存储路径 本地
CSV_PATH:CSV数据库文件存储路径 本地
VSFTPD_PATH:vsftpd上传路径 服务器
CSV_SERVER_PATH:csv文件存储路径 服务器
mysql_status = "Online":mysql状态标志位,主要用来判断mysql是否曾经离线
- "Online": 当前在线
- "offline": 曾经离线









