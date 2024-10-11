#!/usr/bin/env python3
# coding=utf-8

import rospy
import sqlite3
import pymysql
import csv
from ftplib import FTP
from std_msgs.msg import String
from datetime import datetime

config_mysql = {
    "host": "192.168.31.193",  # MySQL服务器的IP
    "user": "star",  # MySQL服务器用户名
    "password": "12345678",  # MySQL服务器密码
    "database": "mysqldata",  # MySQL服务器数据库
    "local_infile": True,  # 允许客户端加载文件
}
config_server = {
    "host": "120.55.164.135",  # MySQL服务器的IP
    "user": "srai",  # MySQL服务器用户名
    "password": "TjcjPDZW76edShYS",  # MySQL服务器密码
    "database": "srai_star_cloud",  # MySQL服务器数据库
}
config_vsftpd = {
    "host": "192.168.31.193",  # vsftpd服务器的IP
    "user": "star",  # vsftpd服务器用户名
    "passwd": "star",  # vsftpd服务器密码
}
SQLITE3_PATH = "/home/star/catkin_ws/src/error_sql_pkg/launch/error.db"
"""
SQLite3数据库文件存储路径 本地
"""
CSV_PATH = "/home/star/catkin_ws/src/error_sql_pkg/launch/error.csv"
"""
CSV文件存储路径 本地
"""
VSFTPD_PATH = "/ros_log/"
"""
vsftpd上传路径 服务器
"""
CSV_SERVER_PATH = f"/home/star{VSFTPD_PATH}error.csv"
"""
csv文件存储路径 服务器
"""
id = []
"""
用于记录sqlite中最后一行数据
"""
mysql_status = "Online"
"""
mysql状态标志位,主要用来判断mysql是否曾经离线

- "Online": 当前在线
- "offline": 曾经离线
"""


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
                error_msg TEXT NOT NULL
            ) COMMENT='"YHS_CIR02/Error" 话题错误日志记录'
            """
            # 执行SQL语句
            cursor.execute(create_table_query)
            # 提交事务
            conn.commit()
            rospy.loginfo("create_mysql_table")
    except pymysql.MySQLError as e:
        rospy.logerr(f"Error: {e}")
    finally:
        conn.close()


def insert_mysql(config_mysql, table_name, time, topic, error_msg):
    """
    往MySQL数据库中的表格写入数据

    :param config_mysql: mysql参数配置
    :param table_name: 操作的表格名
    :param time: 消息时间
    :param topic: 消息话题
    :param error_msg: 消息内容
    """
    try:
        # 使用config_mysql连接到MySQL数据库
        conn = pymysql.connect(**config_mysql)
        # 创建一个cursor对象，with语句确保操作结束后cursor对象自动关闭
        with conn.cursor() as cursor:
            insert_query = f"""
            INSERT INTO {table_name} (time, topic, error_msg) 
            VALUES (%s, %s, %s)
            """
            # 执行SQL语句
            cursor.execute(
                insert_query,
                (
                    time,
                    topic,
                    error_msg,
                ),
            )
            # 提交事务
            conn.commit()
            rospy.loginfo("insert_mysql")
    except pymysql.MySQLError as e:
        rospy.logerr(f"Error: {e}")
    finally:
        conn.close()


def select_mysql_id(config_mysql, table_name):
    """
    查找mysql表中最后一行数据的id

    :param config_mysql: mysql参数配置
    :param table_name: 操作的表格名
    :return: 返回最后一行的数据或者0
    """
    try:
        # 使用config_mysql连接到MySQL数据库
        conn = pymysql.connect(**config_mysql)
        # 创建一个cursor对象，with语句确保操作结束后cursor对象自动关闭
        with conn.cursor() as cursor:
            select_query = f"""
            SELECT id FROM {table_name} ORDER BY id DESC LIMIT 1
            """
            # 执行SQL语句
            cursor.execute(select_query)
            # 获取查询结果
            row = cursor.fetchone()
            rospy.loginfo("select_mysql")
            if row:
                return row[0]  # 返回 ID 值
            else:
                return 0  # 如果没有数据，返回 0
    except pymysql.MySQLError as e:
        rospy.logerr(f"Error: {e}")
        return 0  # 返回 0 表示出错
    finally:
        conn.close()


def check_sqlite3_data(table_name):
    """
    检查sqlite中是否有数据

    :param table_name: 操作的表格名
    """
    try:
        conn = sqlite3.connect(SQLITE3_PATH)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        check_sqlite3_data = f"SELECT EXISTS(SELECT 1 FROM {table_name})"
        # 执行sql语句
        cursor.execute(check_sqlite3_data)
        exists = cursor.fetchone()[0]
        rospy.loginfo("check_sqlite3_data ")
        return exists == 1
    except sqlite3.Error as e:
        rospy.logerr(f"Error: {e}")
        return 0
    finally:
        cursor.close()
        conn.close()


def create_sqlite3_table(table_name):
    """
    创建sqlite文件和表格

    :param table_name: 操作的表格名
    """
    try:
        conn = sqlite3.connect(SQLITE3_PATH)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        create_table_query = f"""
        CREATE TABLE IF NOT EXISTS {table_name} (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            time TEXT NOT NULL,
            topic TEXT NOT NULL,
            error_msg TEXT NOT NULL
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


def insert_sqlite3(table_name, time, topic, error_msg):
    """
    往sqlite表中插入数据

    :param table_name: 操作的表格名
    :param time: 消息时间
    :param topic: 消息话题
    :param error_msg: 消息内容
    """
    try:
        conn = sqlite3.connect(SQLITE3_PATH)
        # 创建一个cursor对象，用来执行SQL语句
        cursor = conn.cursor()
        insert_query = f"""
        INSERT INTO {table_name} (time, topic, error_msg)
        VALUES (?, ?, ?)
        """
        # 执行sql语句
        cursor.execute(
            insert_query,
            (
                time,
                topic,
                error_msg,
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


def delete_sqlite3_data(table_name):
    """
    删除sqlite表中的数据

    :param table_name: 操作的表格名
    """
    conn = sqlite3.connect(SQLITE3_PATH)
    cursor = conn.cursor()
    cursor.execute(f"DELETE FROM {table_name}")
    conn.commit()
    conn.close()
    rospy.loginfo("delete_sqlite3_data")


def select_sqlite3_id(table_name):
    """
    查找sqlite表中最后一行数据的id

    :param table_name: 操作的表格名
    :return: 返回最后一行的数据或者0
    """
    try:
        conn = sqlite3.connect(SQLITE3_PATH)
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


def export_sqlite3_to_csv(table_name, id):
    """
    导出sqlite表的数据到csv文件中

    :param table_name: 操作的表格名
    :param id: sqlite数据导出起始id
    """
    conn = sqlite3.connect(SQLITE3_PATH)
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


def sqlite3_send_mysql(config_mysql, sqlite3_table_name, mysql_table_name, id):
    """
    把sqlite中的数据发送到mysql中

    :param config_mysql: mysql参数配置
    :param sqlite3_table_name: sqlite表格名
    :param mysql_table_name: mysql表格名
    :param id: sqlite数据导出起始id
    """
    """
    自动触发还是计时触发（多线程）放弃
    把sqlite的数据为中间格式(如 CSV 或 JSON)
    1.从sqlite导出 2.上传csv到ftp 3.导入到mysql 
    导出数据的时候，避免应数据太多，导致内存占用问题(后续优化)
    """
    """
    1.从sqlite中导出指定id后的数据
    """
    export_sqlite3_to_csv(sqlite3_table_name, id)
    """
    2.把csv文件上传到vsftpd中
    """
    stor_csv_vsftpd()
    """
    3.导入到mysql,导入的时候跳过文件中的id列
    """
    import_csv_to_mysql(config_mysql, mysql_table_name)


def Error_callback(msg):
    """
    话题订阅回调函数 收到消息后写入到sqlite3文件中,
     通过判断mysql数据库连接状态决定是否写入到mysql中

    :param msg:回调消息
    """
    # 全局变量声明
    global id, mysql_status
    rospy.loginfo("%s", msg.data)
    time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    topic = "YHS_CIR02/Error"
    insert_sqlite3("error_log", time, topic, msg.data)
    """
    在线就直接写入mysql。但是判断到mysql不在线后,立刻查看当前sqlite最后一条记录的id记录并将状态位置为"offline"
    判断到mysql重新上线后把掉线后的记录到的起始id到最后的所有数据通过csv、ftp导入到mysql中
    所有函数都增加了异常抛出,确保不会因为异常中断程序
    只要收到消息就进行在线判断。但如果数据接收频率较高,可能会导致数据丢失。
    可以使用周期判断,实现需要多线程
    """
    if check_mysql_connection(config_mysql):
        if mysql_status == "offline":
            """
            判断到曾经掉线后将掉线前的id到最新的数据全部发送到mysql中,
            此时发送的数据包含了最新准备写入到mysql的数据,所以不需要在重复写入到mysql中
            """
            sqlite3_send_mysql(config_mysql, "error_log", "error_log", id[0])
            # row = select_mysql_id(config_mysql, "error_log")
            # rospy.loginfo("id=%d", row)
            mysql_status = "Online"
        elif mysql_status == "Online":
            # row = select_mysql_id(config_mysql, "error_log")
            # rospy.loginfo("id=%d", row)
            insert_mysql(config_mysql, "error_log", time, topic, msg.data)
    else:
        """
        查看当前sqlite数据库最后一条数据的id,此id行的数据是刚写到slite中的。状态位置为离线
        """
        if mysql_status == "Online":
            """
            仅在mysql离线时的第一次记录id,记录id后将状态为置为"offline"
            mysql上线前都不在更新上传到mysql中的sqlite起始id值
            """
            id = select_sqlite3_id("error_log")
            mysql_status = "offline"


def main():
    rospy.init_node("error_sql_py")
    rospy.Subscriber("YHS_CIR02/Error", String, Error_callback)
    rospy.loginfo("error_sql_py")
    create_sqlite3_table("error_log")
    if check_mysql_connection(config_mysql):
        create_mysql_table(config_mysql, "error_log")
    rospy.spin()


if __name__ == "__main__":
    main()
