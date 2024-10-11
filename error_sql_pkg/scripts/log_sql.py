#!/usr/bin/env python3
# coding=utf-8

import rospy

# from error_sql_pkg.scripts import log_mysql
# from error_sql_pkg.scripts import log_sqlite3
# from error_sql_pkg.scripts import send_sql

import log_mysql
import log_sqlite3
import send_sql
from std_msgs.msg import String, Bool, UInt8
from error_sql_pkg.msg import Battery
from datetime import datetime

SQLITE3_CONFIG_PATH = "/home/star/catkin_ws/src/error_sql_pkg/launch/config.db"
"""
SQLite3 配置数据库文件存储路径 本地
"""


def Stop_callback(msg, args):
    """
    话题"stop"回调函数
    """
    topic = args[0]
    table_name = args[1]
    config_mysql = args[2]
    # rospy.loginfo("topic:%s;", args[0])
    # rospy.loginfo("msg:%s;", msg.data)
    # rospy.loginfo("table_name:%s;", args[1])
    # rospy.loginfo("config_mysql:%s;", args[2])
    rospy.loginfo("topic:%s; msg:%s", topic, msg.data)

    time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    # 写入sqlite
    log_sqlite3.insert_sqlite3(
        log_sqlite3.SQLITE3_LOG_PATH, table_name, time, topic, str(msg.data)
    )
    # 判断mysql的状态 将离线未写入的数据写入到mysql
    send_sql.state_judgement(config_mysql, table_name, time, topic, str(msg.data))


def Error_callback(msg, args):
    """
    话题"YHS_CIR02/Error"回调函数
    """
    topic = args[0]
    table_name = args[1]
    config_mysql = args[2]
    rospy.loginfo("topic:%s; msg:%s", topic, msg.data)
    time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    # 写入sqlite
    log_sqlite3.insert_sqlite3(
        log_sqlite3.SQLITE3_LOG_PATH, table_name, time, topic, str(msg.data)
    )
    # 判断mysql的状态 将离线未写入的数据写入到mysql
    send_sql.state_judgement(config_mysql, table_name, time, topic, str(msg.data))


def Move_callback(msg, args):
    """
    话题"lightcon/movestatus"回调函数
    """
    topic = args[0]
    table_name = args[1]
    config_mysql = args[2]
    rospy.loginfo("topic:%s; msg:%s", topic, msg.data)
    time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    # 写入sqlite
    log_sqlite3.insert_sqlite3(
        log_sqlite3.SQLITE3_LOG_PATH, table_name, time, topic, str(msg.data)
    )
    # 判断mysql的状态 将离线未写入的数据写入到mysql
    send_sql.state_judgement(config_mysql, table_name, time, topic, str(msg.data))


def Battery_callback(msg, args):
    """
    话题"battery_pub"回调函数
    """
    topic = args[0]
    table_name = args[1]
    config_mysql = args[2]
    rospy.loginfo(
        "topic:%s; Electricity:%s; Charging:%s; Power:%s; Capacity:%s; Temperature:%s;",
        topic,
        msg.electricity,
        msg.charging,
        msg.power,
        msg.capacity,
        msg.temperature,
    )
    time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    # msg_data = f"""Electricity={msg.electricity}, Charging={msg.charging}, Power={msg.power}, Capacity={msg.capacity}, Temperature={msg.temperature}"""
    # 写入sqlite
    log_sqlite3.insert_sqlite3(
        log_sqlite3.SQLITE3_LOG_PATH, table_name, time, topic, str(msg)
    )
    # 判断mysql的状态 将离线未写入的数据写入到mysql
    send_sql.state_judgement(config_mysql, table_name, time, topic, str(msg))


def WakeUp_callback(msg, args):
    """
    话题"aikit/wakenUp"回调函数
    """
    topic = args[0]
    table_name = args[1]
    config_mysql = args[2]
    rospy.loginfo("topic:%s; msg:%s", topic, msg.data)
    time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    # 写入sqlite
    log_sqlite3.insert_sqlite3(
        log_sqlite3.SQLITE3_LOG_PATH, table_name, time, topic, str(msg.data)
    )
    # 判断mysql的状态 将离线未写入的数据写入到mysql
    send_sql.state_judgement(config_mysql, table_name, time, topic, str(msg.data))


def main():
    rospy.init_node("log_sql_py")
    rospy.loginfo("log_sql_py")
    # 获取SQLITE3_LOG_PATH字段
    log_sqlite3.select_sqlite3(
        SQLITE3_CONFIG_PATH,
        "file",
        "name",
        "SQLITE3_LOG_PATH",
    )
    # 获取SQLITE3_MIDDLE_PATH字段
    log_sqlite3.select_sqlite3(
        SQLITE3_CONFIG_PATH,
        "file",
        "name",
        "SQLITE3_MIDDLE_PATH",
    )
    send_sql.SQLITE3_LOG_PATH = log_sqlite3.SQLITE3_LOG_PATH
    # 获取mysql配置信息
    log_mysql.select_sqlite3(SQLITE3_CONFIG_PATH, "server", "type", "mysql")
    # 获取vsftp配置信息
    send_sql.select_sqlite3(SQLITE3_CONFIG_PATH, "server", "type", "ftp")
    # 获取中间文件位置
    send_sql.select_sqlite3(
        SQLITE3_CONFIG_PATH,
        "file",
        "name",
        "CSV_PATH",
    )
    send_sql.select_sqlite3(
        SQLITE3_CONFIG_PATH,
        "file",
        "name",
        "VSFTPD_PATH",
    )
    send_sql.select_sqlite3(
        SQLITE3_CONFIG_PATH,
        "file",
        "name",
        "CSV_SERVER_PATH",
    )

    operation_table = "operation_log"
    error_table = "error_log"
    config_mysql = log_mysql.config_mysql
    # 创建运行状态表格 operation_log 和错误日志表格 error_log
    log_sqlite3.create_sqlite3_table(log_sqlite3.SQLITE3_LOG_PATH, operation_table)
    log_sqlite3.create_sqlite3_table(log_sqlite3.SQLITE3_LOG_PATH, error_table)
    # 创建middle.db表格
    log_sqlite3.create_sqlite3_table(log_sqlite3.SQLITE3_MIDDLE_PATH, "operation_log")
    log_sqlite3.create_sqlite3_table(log_sqlite3.SQLITE3_MIDDLE_PATH, "error_log")
    # 检查mysql的连接,创建mysql表格
    if log_mysql.check_mysql_connection(config_mysql):
        log_mysql.create_mysql_table(config_mysql, operation_table)
        log_mysql.create_mysql_table(config_mysql, error_table)

    # 急停 true:急停 false:非急停 callback_args传递额外参数
    rospy.Subscriber(
        "stop",
        Bool,
        Stop_callback,
        callback_args=("stop", operation_table, config_mysql),
    )
    # 错误、故障信息
    rospy.Subscriber(
        "YHS_CIR02/Error",
        String,
        Error_callback,
        callback_args=("YHS_CIR02/Error", error_table, config_mysql),
    )
    # 移动 0:待机 1:导航移动 2:手柄移动
    rospy.Subscriber(
        "lightcon/movestatus",
        UInt8,
        Move_callback,
        callback_args=("lightcon/movestatus", operation_table, config_mysql),
    )
    # 电池 自定义消息
    rospy.Subscriber(
        "battery_pub",
        Battery,
        Battery_callback,
        callback_args=("battery_pub", operation_table, config_mysql),
    )
    # 语言唤醒 true:已唤醒 false:未唤醒
    rospy.Subscriber(
        "aikit/wakenUp",
        String,
        WakeUp_callback,
        callback_args=("aikit/wakenUp", operation_table, config_mysql),
    )

    rospy.spin()


if __name__ == "__main__":
    main()
