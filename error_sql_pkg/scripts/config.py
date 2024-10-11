#!/usr/bin/env python3
# coding=utf-8

import rospy
import sqlite3


def create_sqlite3_server(path):
    """
    创建sqlite文件和表格

    :param path: 数据库文件路径
    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        create_table_query = f"""
        CREATE TABLE IF NOT EXISTS server (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            type TEXT NOT NULL CHECK(type IN ('mqtt', 'tcp', 'stream', 'ftp', 'mysql')) UNIQUE,
            host VARCHAR(255) NOT NULL,            -- 服务器 地址
            username VARCHAR(100),                 -- 用户名
            password VARCHAR(100),                 -- 密码
            client_id VARCHAR(100),                -- 客户端ID(特定于 mqtt)
            database_name VARCHAR(100),            -- 数据库名称（特定于 mysql)
            additional_info TEXT                   -- 额外信息
        )
            """
        # 执行sql语句
        cursor.execute(create_table_query)
        # 提交事务
        conn.commit()
        rospy.loginfo("create_sqlite3_table")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def insert_sqlite3_server(
    path,
    type,
    host,
    username,
    password,
    client_id=None,
    database_name=None,
    additional_info=None,
):
    """
    往sqlite表中插入数据

    :param path: 数据库文件路径
    :param type: 字段类型
    :param host: IP地址或者URL
    :param username: 用户名
    :param password: 密码
    :param client_id: 可选 MQTT客户端ID
    :param database_name: 可选 MYSQL可操作的表格名
    :param additional_info: 可选 额外字段

    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        insert_query = f"""
        INSERT INTO server (type, host, username, password, client_id, database_name, additional_info)
        VALUES (?, ?, ?, ?, ?, ?, ?)
        """
        # 执行sql语句
        cursor.execute(
            insert_query,
            (type, host, username, password, client_id, database_name, additional_info),
        )
        # 提交事务
        conn.commit()
        rospy.loginfo("insert_sqlite")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def create_sqlite3_network(path):
    """
    创建sqlite文件和表格

    :param path: 数据库文件路径
    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        create_table_query = f"""
        CREATE TABLE IF NOT EXISTS network (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            type TEXT NOT NULL CHECK(type IN ('wifi', 'hotspot')) UNIQUE,
            SSID VARCHAR(100) NOT NULL,            -- 名称
            password VARCHAR(100) NOT NULL,        -- 密码
            agreement VARCHAR(100),                -- 协议
            additional_info TEXT                   -- 额外信息
        )
            """
        # 执行sql语句
        cursor.execute(create_table_query)
        # 提交事务
        conn.commit()
        rospy.loginfo("create_sqlite3_table")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def insert_sqlite3_network(
    path,
    type,
    SSID,
    password,
    agreement=None,
    additional_info=None,
):
    """
    往sqlite表中插入数据

    :param path: 数据库文件路径
    :param type: 字段类型
    :param SSID: 网络名称
    :param password: 密码
    :param agreement: 可选 网络协议
    :param additional_info: 可选 额外字段

    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        insert_query = f"""
        INSERT INTO network (type, SSID, password, agreement, additional_info)
        VALUES (?, ?, ?, ?, ?)
        """
        # 执行sql语句
        cursor.execute(
            insert_query,
            (
                type,
                SSID,
                password,
                agreement,
                additional_info,
            ),
        )
        # 提交事务
        conn.commit()
        rospy.loginfo("insert_sqlite")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def create_sqlite3_user(path):
    """
    创建sqlite文件和表格

    :param path: 数据库文件路径
    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        create_table_query = f"""
        CREATE TABLE IF NOT EXISTS user (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            user_id VARCHAR(100) NOT NULL UNIQUE,    -- 用户ID
            user_map VARCHAR(100) NOT NULL,          -- 用户地图
            user_SSID VARCHAR(100) NOT NULL,         -- 用户网络名称
            password VARCHAR(100) NOT NULL,          -- 用户网络密码
            additional_info TEXT                     -- 额外信息
        )
            """
        # 执行sql语句
        cursor.execute(create_table_query)
        # 提交事务
        conn.commit()
        rospy.loginfo("create_sqlite3_table")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def insert_sqlite3_user(
    path,
    user_id,
    user_map,
    user_SSID,
    password,
    additional_info=None,
):
    """
    往sqlite表中插入数据

    :param path: 数据库文件路径
    :param user_id: 用户ID
    :param user_map: 用户地图
    :param user_SSID: 用户网络名称
    :param password: 用户网络密码
    :param additional_info: 可选 额外字段

    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        insert_query = f"""
        INSERT INTO user (user_id, user_map, user_SSID, password, additional_info)
        VALUES (?, ?, ?, ?, ?)
        """
        # 执行sql语句
        cursor.execute(
            insert_query,
            (
                user_id,
                user_map,
                user_SSID,
                password,
                additional_info,
            ),
        )
        # 提交事务
        conn.commit()
        rospy.loginfo("insert_sqlite")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def create_sqlite3_file(path):
    """
    创建sqlite文件和表格

    :param path: 数据库文件路径
    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        create_table_query = f"""
        CREATE TABLE IF NOT EXISTS file (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name VARCHAR(100) NOT NULL UNIQUE,     -- 名称
            file_path VARCHAR(255) NOT NULL,       -- 文件路径
            explain VARCHAR(100) NOT NULL,         -- 说明
            additional_info TEXT                   -- 额外信息
        )
            """
        # 执行sql语句
        cursor.execute(create_table_query)
        # 提交事务
        conn.commit()
        rospy.loginfo("create_sqlite3_table")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def insert_sqlite3_file(
    path,
    name,
    file_path,
    explain,
    additional_info=None,
):
    """
    往sqlite表中插入数据

    :param path: 数据库文件路径
    :param name: 名称
    :param file_path: 文件路径
    :param explain:描述
    :param additional_info: 可选 额外字段

    """
    try:
        conn = sqlite3.connect(path)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        insert_query = f"""
        INSERT INTO file (name, file_path, explain, additional_info)
        VALUES (?, ?, ?, ?)
        """
        # 执行sql语句
        cursor.execute(
            insert_query,
            (
                name,
                file_path,
                explain,
                additional_info,
            ),
        )
        # 提交事务
        conn.commit()
        rospy.loginfo("insert_sqlite")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def main():
    SQLITE3_PATH = "/home/star/catkin_ws/src/error_sql_pkg/launch/config.db"
    """
    SQLite3数据库文件存储路径 本地
    """
    # 创建数据库和表格
    create_sqlite3_server(SQLITE3_PATH)
    create_sqlite3_network(SQLITE3_PATH)
    create_sqlite3_user(SQLITE3_PATH)
    create_sqlite3_file(SQLITE3_PATH)

    # MQTT字段
    insert_sqlite3_server(
        SQLITE3_PATH,
        "mqtt",
        "mqtt://120.55.164.135:1883",
        "srai_client",
        "srai_client",
        client_id="SRAI_time",
    )
    # TCP字段
    insert_sqlite3_server(
        SQLITE3_PATH,
        "tcp",
        "tcp://120.55.164.135:1883",
        "srai_client",
        "srai_client",
    )
    # 流媒体字段
    insert_sqlite3_server(
        SQLITE3_PATH,
        "stream",
        "stream://120.55.164.135:1883",
        "srai_client",
        "srai_client",
    )
    # ftp字段
    insert_sqlite3_server(
        SQLITE3_PATH,
        "ftp",
        "ftp://192.168.31.193",
        "star",
        "star",
    )
    # MQSQL字段
    insert_sqlite3_server(
        SQLITE3_PATH,
        "mysql",
        "192.168.31.193",
        "star",
        "12345678",
        database_name="mysqldata",
        additional_info="True",
    )
    # wifi字段
    insert_sqlite3_network(
        SQLITE3_PATH,
        "wifi",
        "star",
        "12345678",
    )
    # hotspot字段
    insert_sqlite3_network(
        SQLITE3_PATH,
        "hotspot",
        "star",
        "12345678",
    )
    # 用户配置
    insert_sqlite3_user(
        SQLITE3_PATH, "star", "home/star/map/2024-09-26.bag", "star", "12345678"
    )
    # SQLITE3_LOG_PATH字段
    insert_sqlite3_file(
        SQLITE3_PATH,
        "SQLITE3_LOG_PATH",
        "/home/star/catkin_ws/src/error_sql_pkg/launch/log.db",
        "日志文件本地路径",
    )
    # SQLITE3_LOG_PATH字段
    insert_sqlite3_file(
        SQLITE3_PATH,
        "SQLITE3_MIDDLE_PATH",
        "/home/star/catkin_ws/src/error_sql_pkg/launch/middle.db",
        "保存mysql掉线期间的日志本地路径",
    )
    # CSV_PATH字段
    insert_sqlite3_file(
        SQLITE3_PATH,
        "CSV_PATH",
        "/home/star/catkin_ws/src/error_sql_pkg/launch/log.csv",
        "CSV文件本地路径",
    )
    # VSFTPD_PATH字段
    insert_sqlite3_file(
        SQLITE3_PATH, "VSFTPD_PATH", "/ros_log/", "vsftpd上传到服务器的路径"
    )
    insert_sqlite3_file(
        SQLITE3_PATH,
        "CSV_SERVER_PATH",
        "/home/star{VFTPD_PATH}log.csv",
        "csv文件存储路径 服务器",
    )


if __name__ == "__main__":
    main()
