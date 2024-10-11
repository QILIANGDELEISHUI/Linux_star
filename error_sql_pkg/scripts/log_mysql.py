#!/usr/bin/env python3
# coding=utf-8

import rospy
import sqlite3
import pymysql
import log_sql


config_mysqll = {
    "host": "192.168.31.193",  # MySQL服务器的IP
    "user": "star",  # MySQL服务器用户名
    "password": "12345678",  # MySQL服务器密码
    "database": "mysqldata",  # MySQL服务器数据库
    "local_infile": True,  # 允许客户端加载文件
}
config_mysql = {}
"""
mysql配置信息
"""

def select_sqlite3(path,table_name, colums_name, name):
    """
    查询字段值

    :param path: 操作数据库的路径
    :param table_name: 表格名
    :param colums_name: 列名称
    :param name: 字段名
    """
    global config_mysql
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        select_table_query = f"""
        SELECT * FROM {table_name} WHERE {colums_name} = "{name}" 
            """
        # 执行sql语句
        cursor.execute(select_table_query)
        # 获取结果
        results = cursor.fetchall()
        # 打印结果
        for row in results:
            print(
                f"ID: {row[0]}\n"
                f"type: {row[1]}\n"
                f"host: {row[2]}\n"
                f"username: {row[3]}\n"
                f"password: {row[4]}\n"
                f"database: {row[6]}\n"
                f"local_infile: {row[7]}\n"
            )
        config_mysql = {
            "host": row[2],  # MySQL服务器的IP
            "user": row[3],  # MySQL服务器用户名
            "password": row[4],  # MySQL服务器密码
            "database": row[6],  # MySQL服务器数据库
            "local_infile": row[7],  # 允许客户端加载文件
        }
        # rospy.loginfo("select_sqlite3")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def check_mysql_connection(config_mysql):
    """
    检查MySQL数据库的连接

    :param config_mysql: mysql参数配置
    """
    conn = None
    try:
        conn = pymysql.connect(**config_mysql)
        # 创建一个cursor对象，with语句确保操作结束后cursor对象自动关闭
        with conn.cursor() as cursor:
            cursor.execute("SELECT 1")
            results = cursor.fetchone()
            if results is not None and results[0] == 1:
                return True
    except pymysql.MySQLError as e:
        rospy.logerr(f"Error: {e}")
        return False
    finally:
        if conn:
            conn.close()


def create_mysql_table(config_mysql, table_name):
    """
    创建或连接MySQL数据库中的表格

    :param config_mysql: mysql参数配置
    :param table_name: 表格名
    """
    try:
        # 使用config_mysql连接到MySQL数据库

        conn = pymysql.connect(**config_mysql)
        # 创建一个cursor对象，with语句确保操作结束后cursor对象自动关闭
        with conn.cursor() as cursor:
            create_table_query = f"""
            CREATE TABLE IF NOT EXISTS {table_name} (
                id INT AUTO_INCREMENT PRIMARY KEY,
                time VARCHAR(255) NOT NULL,
                topic VARCHAR(255) NOT NULL,
                msg TEXT NOT NULL
            )
            """
            # 执行SQL语句
            cursor.execute(create_table_query)
            # 提交事务
            conn.commit()
            rospy.loginfo(f"create_mysql_table:{table_name}")
    except pymysql.MySQLError as e:
        rospy.logerr(f"Error: {e}")
    finally:
        conn.close()


def insert_mysql(config_mysql, table_name, time, topic, msg):
    """
    往MySQL数据库中的表格写入数据

    :param config_mysql: mysql参数配置
    :param table_name: 操作的表格名
    :param time: 消息时间
    :param topic: 消息话题
    :param msg: 消息内容
    """
    try:
        # 使用config_mysql连接到MySQL数据库
        conn = pymysql.connect(**config_mysql)
        # 创建一个cursor对象，with语句确保操作结束后cursor对象自动关闭
        with conn.cursor() as cursor:
            insert_query = f"""
            INSERT INTO {table_name} (time, topic, msg) 
            VALUES (%s, %s, %s)
            """
            # 执行SQL语句
            cursor.execute(
                insert_query,
                (
                    time,
                    topic,
                    msg,
                ),
            )
            # 提交事务
            conn.commit()
            rospy.loginfo("insert_mysql")
    except pymysql.MySQLError as e:
        rospy.logerr(f"Error: {e}")
    finally:
        conn.close()
