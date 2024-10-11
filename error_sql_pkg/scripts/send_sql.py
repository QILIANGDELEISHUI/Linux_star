#!/usr/bin/env python3
# coding=utf-8

import rospy
import sqlite3
import pymysql
import log_sql
import log_mysql
import log_sqlite3
import csv
from ftplib import FTP


SQLITE3_LOG_PATH = None
"""
SQLite3数据库文件存储路径 本地
"""
CSV_PATH = None
"""
CSV文件存储路径 本地
"""
VSFTPD_PATH = None
"""
vsftpd上传路径 服务器
"""
CSV_SERVER_PATH = None
"""
csv文件存储路径 服务器
"""
config_vsftpd = {
    "host": "192.168.31.193",  # vsftpd服务器的IP
    "user": "star",  # vsftpd服务器用户名
    "passwd": "star",  # vsftpd服务器密码
}
mysql_status = "Online"
"""
mysql状态标志位,主要用来判断mysql是否曾经离线

- "Online": 当前在线
- "offline": 曾经离线
"""


def select_sqlite3(path, table_name, colums_name, name):
    """
    查询字段值

    :param path: 操作数据库的路径
    :param table_name: 表格名
    :param colums_name: 列名称
    :param name: 字段名
    """
    global CSV_PATH, VSFTPD_PATH, CSV_SERVER_PATH, config_vsftpd
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
                f"ID: {row[0]}\nname: {row[1]}\nfile_path: {row[2]}\nexplain: {row[3]}"
            )
        if name == "CSV_PATH":
            CSV_PATH = row[2]
            print(
                f"CSV_PATH:{CSV_PATH}\n",
            )
        elif name == "VSFTPD_PATH":
            VSFTPD_PATH = row[2]
            print(
                f"VSFTPD_PATH:{VSFTPD_PATH}\n",
            )
        elif name == "CSV_SERVER_PATH":
            CSV_SERVER_PATH = row[2].replace("{VFTPD_PATH}", VSFTPD_PATH)
            print(
                f"CSV_SERVER_PATH:{CSV_SERVER_PATH}\n",
            )
        elif name == "ftp":
            config_vsftpd = {
                "host": row[2],  # vsftpd服务器的IP
                "user": row[3],  # vsftpd服务器用户名
                "passwd": row[4],  # vsftpd服务器密码
            }
            print(f"host: {row[2]}\n" f"user: {row[3]}\n" f"passwd: {row[4]}\n")
        # rospy.loginfo("select_sqlite3")
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
    finally:
        cursor.close()
        conn.close()


def export_sqlite3_to_csv(table_name, id):
    """
    导出sqlite表的数据到csv文件中

    :param table_name: 操作的表格名
    :param id: sqlite数据导出起始id
    """
    conn = sqlite3.connect(SQLITE3_LOG_PATH)
    cursor = conn.cursor()
    # 查询表中指定id后的所有数据，查询不需要提交事务
    export_sqlite3_query = f"SELECT * FROM {table_name} WHERE id >= {id}"
    cursor.execute(export_sqlite3_query)
    """
    将查询到的所有数据保存列表中，列表中的元素构成是元组，一个元组就是一行数据
    当表中数据比较多的话，需要修改这部分代码，逐行解析或者一只次提取部分数据
    此处使用将全部数据提取,包括ID行,插入到mysql中跳过ID列。
    也可以在提取数据时,跳过ID列,但可能需要同步修改import_csv_to_mysql();
    """
    rows = cursor.fetchall()

    with open(CSV_PATH, "w", newline="", encoding="utf-8") as csvfile:
        """
        以"w"写的方式打开csv文件,如果源文件存在则会覆盖,行结束符为空,确保不会插入额外空行,编码格式为utf-8
        """
        # 创建一个csv写入器
        writer = csv.writer(csvfile)
        """
        写入列名 cursor.description用于获取查询结果集的列的元数据,包括每列的名称、类型等
        desc[0] for desc in cursor.description  列表推导式
        依次把cursor.description中的数据(元组)赋值给desc desc[0]指的是desc元组中的第一个数据
        遍历cursor.description结果集,获取每一个结果集(元组)中的第一个数据，这通常是列名
        writer.writerow(...)，一次写入一行
        """
        writer.writerow([desc[0] for desc in cursor.description])
        # writer.writerows(...) 写入多行数据 参数为包含多行数据的可迭代对象
        writer.writerows(rows)
        rospy.loginfo("export_sqlite3_to_csv")

    cursor.close()
    conn.close()


def stor_csv_vsftpd():
    """
    保存csv文件到vsftpd
    """
    try:
        # 连接到 FTP 服务器
        ftp = FTP(config_vsftpd["host"])
        ftp.login(user=config_vsftpd["user"], passwd=config_vsftpd["passwd"])

        # 跳转到指定目录
        ftp.cwd(VSFTPD_PATH)
        # 打开要上传的文件
        with open(CSV_PATH, "rb") as file:
            # 使用 FTP 的 STOR 命令上传文件
            ftp.storbinary(f'STOR {CSV_PATH.split("/")[-1]}', file)
    except Exception as e:
        rospy.loginfo(f"上传文件时发生错误: {e}")
    finally:
        # 关闭 FTP 连接
        ftp.quit()
        rospy.loginfo(
            f"文件 {CSV_PATH.split('/')[-1]} 已成功上传到 {config_vsftpd['host']}"
        )


def import_csv_to_mysql(config_mysql, table_name):
    """
    导入csv文件到mysql表中

    :param config_mysql: mysql参数配置
    :param table_name: 操作的表格名
    """
    conn = None
    try:
        # 使用config_mysql连接到MySQL数据库
        conn = pymysql.connect(**config_mysql)
        # 创建一个cursor对象，with语句确保操作结束后cursor对象自动关闭
        with conn.cursor() as cursor:
            """
            使用LOAD DATA LOCAL INFILE导入CSV文件指定了local,则表示文件位于客户端,如果没有,则表示文件在Server端。
            插入到table_name表格
            字段的中止符是 ,
            字段内容可以使用"来包围
            记录的终止符是\n
            加载时忽略文件的第一行,即标题
            使用@dummy来跳过指定行
            """
            query = f"""
            LOAD DATA LOCAL INFILE '{CSV_SERVER_PATH}'
            INTO TABLE {table_name}
            FIELDS TERMINATED BY ','
            OPTIONALLY ENCLOSED BY '"'
            LINES TERMINATED BY '\n'
            IGNORE 1 ROWS
            (@dummy,time, topic, error_msg);
            """
            cursor.execute(query)
            conn.commit()
            rospy.loginfo("import_csv_to_mysql")
    except pymysql.MySQLError as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if conn:
            conn.close()


def sqlite3_send_mysql(sqlite3_path, config_mysql):
    """
    把sqlite中的数据发送到mysql中

    :param sqlite3_path: sqlite3文件路径
    :param config_mysql: mysql参数配置
    """
    try:
        # 连接到 SQLite3 数据库
        sqlite3_conn = sqlite3.connect(sqlite3_path)
        sqlite3_cursor = sqlite3_conn.cursor()
        # 连接到mysql
        mysql_conn = pymysql.connect(**config_mysql)
        with mysql_conn.cursor() as mysql_cursor:
            # 查询 SQLite3 中的所有表
            sqlite3_cursor.execute(
                "SELECT name FROM sqlite_master WHERE type='table' AND name != 'sqlite_sequence';"
            )
            tables = sqlite3_cursor.fetchall()

            # 遍历每个表
            for (table_name,) in tables:
                # 查询当前表的数据
                sqlite3_cursor.execute(f"SELECT * FROM {table_name};")
                rows = sqlite3_cursor.fetchall()

                # 插入数据到 MySQL
                for row in rows:
                    # print(row[1], row[2], row[3])
                    insert_query = f"""
                    INSERT INTO {table_name} (time, topic, msg) 
                    VALUES (%s, %s, %s)
                    """
                    # 执行SQL语句
                    mysql_cursor.execute(
                        insert_query,
                        (
                            row[1],
                            row[2],
                            row[3],
                        ),
                    )
            # 提交事务
            mysql_conn.commit()
            rospy.loginfo("sqlite3_send_mysql")
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


def select_sqlite3_id(table_name):
    """
    查找sqlite表中最后一行数据的id

    :param table_name: 操作的表格名
    :return: 返回最后一行的数据或者0
    """
    try:
        conn = sqlite3.connect(SQLITE3_LOG_PATH)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        select_query = f"""
        SELECT id FROM {table_name} ORDER BY id DESC LIMIT 1
        """
        # 执行sql语句
        cursor.execute(select_query)
        row = cursor.fetchall()
        rospy.loginfo("select_sqlite")
        if row:
            return row[0]  # 返回 ID 值
        else:
            return 0  # 如果没有数据，返回 0
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
        return 0  # 返回 None 表示出错
    finally:
        cursor.close()
        conn.close()


def state_judgement(config_mysql, table_name, time, topic, msg):
    """
    使用状态符判断mysql状态,首次离线后生成middle.db文件,同步记录mysql掉线期间的所有日志消息,
    当检测到mysql上线后,且状态符为"offline" 曾经掉线,就遍历middle.db中的所有表格和数据(除ID列)并插入到mysql
    """
    # 全局变量声明
    global mysql_status
    if log_mysql.check_mysql_connection(config_mysql):
        # 在线
        # 曾经离线
        if mysql_status == "offline":
            """
            判断到曾经掉线后将middle中的数据全部发送到mysql
            此时有一条消息还没有写入,可以先写到middle中或者最后直接发送到mysql中
            """
            """
            掉线后上线
            1.发送临时过渡db文件
            2.读取临时过渡文件
            3.删或不删 vsftp或本地临时过渡db文件 
            """
            mysql_status = "Online"
            # 从临时db文件写入到mysql
            sqlite3_send_mysql(log_sqlite3.SQLITE3_MIDDLE_PATH, config_mysql)
            # 删除数据
            log_sqlite3.delete_sqlite3(log_sqlite3.SQLITE3_MIDDLE_PATH)
            # 将此时的消息写入到mysql
            log_mysql.insert_mysql(config_mysql, table_name, time, topic, msg)
        # 当前在线
        elif mysql_status == "Online":
            """
            一直在线
            """
            log_mysql.insert_mysql(config_mysql, table_name, time, topic, msg)
    else:
        # 不在线
        """
        查看当前sqlite数据库最后一条数据的id,此id行的数据是刚写到sqlite中的。状态位置为离线
        """
        # 当前在线
        if mysql_status == "Online":
            """
            首次离线
            1.获取SQLITE3_MIDDLE_PATH
            2.创建表格
            3.写入数据
            4.更改状态位
            """
            mysql_status = "offline"
        # 写入数据
        log_sqlite3.insert_sqlite3(
            log_sqlite3.SQLITE3_MIDDLE_PATH, table_name, time, topic, msg
        )
