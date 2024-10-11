#!/usr/bin/env python3
# coding=utf-8

import rospy
import sqlite3


SQLITE3_LOG_PATH = None
"""
SQLite3日志数据库文件存储路径 本地
"""
SQLITE3_MIDDLE_PATH = None
"""
保存mysql掉线期间的SQLite3数据库文件存储路径 本地
"""


def select_sqlite3(path, table_name, colums_name, name):
    """
    查询字段值

    :param path: 操作数据库的路径
    :param table_name: 表格名
    :param colums_name: 列名称
    :param name: 字段名
    """
    global SQLITE3_LOG_PATH, SQLITE3_MIDDLE_PATH
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
                f"ID: {row[0]}\nname: {row[1]}\nfile_path: {row[2]}\nexplain: {row[3]}\n"
            )
        if name == "SQLITE3_LOG_PATH":
            SQLITE3_LOG_PATH = row[2]
        elif name == "SQLITE3_MIDDLE_PATH":
            SQLITE3_MIDDLE_PATH = row[2]
        # rospy.loginfo("select_sqlite3")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def create_sqlite3_table(path, table_name):
    """
    创建sqlite文件和表格

    :param path: 操作数据库的路径
    :param table_name: 操作的表格名
    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        create_table_query = f"""
        CREATE TABLE IF NOT EXISTS {table_name} (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            time TEXT NOT NULL,
            topic TEXT NOT NULL,
            msg TEXT NOT NULL
        )
            """
        # 执行sql语句
        cursor.execute(create_table_query)
        # 提交事务
        conn.commit()
        rospy.loginfo("create_sqlite3_table:%s", table_name)
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def insert_sqlite3(path, table_name, time, topic, msg):
    """
    往sqlite表中插入数据

    :param path: 操作数据库的路径
    :param table_name: 操作的表格名
    :param time: 消息时间
    :param topic: 消息话题
    :param msg: 消息内容
    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        insert_query = f"""
        INSERT INTO {table_name} (time, topic, msg)
        VALUES (?, ?, ?)
        """
        # 执行sql语句
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
        filename = path.split("/")[-1]
        rospy.loginfo("insert_sqlite3: %s", filename)
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def delete_sqlite3(path):
    """
    删除sqlite表的全部数据

    :param path: 操作数据库的路径
    """
    try:
        conn = sqlite3.connect(path)
        cursor = conn.cursor()
        # 获取sqlite3全部表格
        cursor.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name != 'sqlite_sequence';"
        )
        tables = cursor.fetchall()
        # 遍历每个表
        for (table_name,) in tables:
            delete_query = f"DELETE FROM {table_name};"
            cursor.execute(delete_query)
            # 提交事务
        conn.commit()
        rospy.loginfo("delete_sqlite3")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()
