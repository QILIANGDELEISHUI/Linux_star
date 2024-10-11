#!/usr/bin/env python3
# coding=utf-8
import csv
import rospy
import sqlite3
import pymysql
import log_mysql
import log_sql
from ftplib import FTP
import sqlite3
import mysql.connector
from mysql.connector import Error

config_mysql = {
    "host": "192.168.31.193",  # MySQL服务器的IP
    "user": "star",  # MySQL服务器用户名
    "password": "12345678",  # MySQL服务器密码
    "database": "mysqldata",  # MySQL服务器数据库
    "local_infile": True,  # 允许客户端加载文件
}

config_vsftpd = {
    "host": "192.168.31.193",  # vsftpd服务器的IP
    "user": "star",  # vsftpd服务器用户名
    "passwd": "star",  # vsftpd服务器密码
}
SQLITE3_PATH = "/home/star/catkin_ws/src/error_sql_pkg/launch/config.db"
"""
SQLite3数据库文件存储路径 本地
"""
SQLITE3_LOG_PATH = "/home/star/catkin_ws/src/error_sql_pkg/launch/log.db"
"""
SQLite3数据库文件存储路径 本地
"""
CSV_PATH = None
"""
CSV数据库文件存储路径
"""
VSFTPD_PATH = None
"""
vsftpd上传路径 服务器
"""
CSV_SERVER_PATH = None
"""
csv文件存储路径 服务器
"""
SQLITE3_LOG_PATH = "/home/star/catkin_ws/src/error_sql_pkg/launch/log.db"
"""
SQLite3 配置数据库文件存储路径 本地
"""

# config_mysql = {}


def sqlite3_send_mysql(sqlite3_path, config_mysql):
    try:
        # 连接到 SQLite3 数据库
        sqlite3_conn = sqlite3.connect(sqlite3_path)
        sqlite3_cursor = sqlite3_conn.cursor()
        # 连接到mysql
        mysql_conn = pymysql.connect(**config_mysql)
        with mysql_conn.cursor() as mysql_cursor:
            # 查询 SQLite3 中的所有表
            sqlite3_cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
            tables = sqlite3_cursor.fetchall()

            # 遍历每个表
            for (table_name,) in tables:
                # 跳过sqlite_sequence表格 sqlite_sequence表格时sqlite3统计表格，非数据
                if table_name == "sqlite_sequence":
                    continue
                print(f"Processing table: {table_name}")

                # 查询当前表的数据
                sqlite3_cursor.execute(f"SELECT * FROM {table_name};")
                rows = sqlite3_cursor.fetchall()

                # 插入数据到 MySQL
                for row in rows:
                    print(row[1], row[2], row[3])
                    log_mysql.insert_mysql(config_mysql, table_name, row[1], row[2], row[3])
            # 提交事务
            mysql_conn.commit()
    except pymysql.MySQLError as e:
        print(f"Error: {e}")
    except sqlite3.Error as e:
        print(f"Error: {e}")
    finally:
        # 关闭连接
        sqlite3_cursor.close()
        sqlite3_conn.close()
        if mysql_conn:
            mysql_conn.close()


def cs_sqlite3(sqlite3_path, config_mysql):
    # 连接到 SQLite3 数据库
    sqlite3_conn = sqlite3.connect(sqlite3_path)
    sqlite3_cursor = sqlite3_conn.cursor()
    # 查询 SQLite3 中的所有表
    sqlite3_cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = sqlite3_cursor.fetchall()
    print(tables)
    # 遍历每个表
    for (table_name,) in tables:
        if table_name == "sqlite_sequence":
            continue
        print(table_name)
        sqlite3_cursor.execute(f"PRAGMA table_info({table_name});")
        columns_info = sqlite3_cursor.fetchall()
        print(columns_info)

        # 提取列名，确定 id 列的位置 结果为[1, 2, 3]
        columns = [column[1] for column in columns_info]  # column[1] 是列名
        if "id" in columns:
            id_column_index = columns.index("id")
            other_column_indexes = [
                i for i in range(len(columns)) if i != id_column_index
            ]
            print(other_column_indexes)
        else:
            other_column_indexes = list(range(len(columns)))
            print(other_column_indexes)

        # 查询当前表的数据
        sqlite3_cursor.execute(f"SELECT * FROM {table_name};")
        rows = sqlite3_cursor.fetchall()
        print(rows)
        # 插入数据到 MySQL
        for row in rows:
            print(row[1], row[2], row[3])
            # log_mysql.insert_mysql(config_mysql, table_name, row[1], row[2], row[3])


if __name__ == "__main__":
    sqlite3_send_mysql(SQLITE3_LOG_PATH, config_mysql)
    # cs_sqlite3(SQLITE3_CONFIG_PATH, config_mysql)
