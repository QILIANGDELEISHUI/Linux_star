#!/usr/bin/env python3
# coding=utf-8
import rospy
from std_msgs.msg import String, Bool, UInt8
from error_sql_pkg.msg import Battery

def stop():
    stop_msg = Bool()
    stop_msg.data = True  # 或 False
    rospy.loginfo(f"Published to stop: {stop_msg.data}")
    pub_stop.publish(stop_msg)  # 发布消息


def error():
    error_msg = String()
    error_msg.data = "No errors detected."  # 或者具体错误信息
    rospy.loginfo(f"Published error message: {error_msg.data}")
    pub_error.publish(error_msg)  # 发布消息


def move():
    move_msg = UInt8()
    move_msg.data = 1  # 可能的状态码，例如 0: 停止, 1: 移动中, 2:完成
    rospy.loginfo(f"Published move status: {move_msg.data}")
    pub_move.publish(move_msg)  # 发布消息


def battery():
    battery_msg = Battery()
    battery_msg.electricity = 75.0  # 电量百分比
    battery_msg.charging = False  # 充电状态
    battery_msg.power = 12.5  # 电池功率
    battery_msg.capacity = 1000.0  # 电池容量，单位 mAh
    battery_msg.temperature = 25.0  # 电池温度，单位摄氏度
    rospy.loginfo(
        f"""Published battery status: 
        Electricity={battery_msg.electricity}, Charging={battery_msg.charging}, Power={battery_msg.power}, Capacity={battery_msg.capacity}, Temperature={battery_msg.temperature}"""
    )
    pub_battery.publish(battery_msg)  # 发布消息


def wakeup():
    wakeup_msg = String()
    wakeup_msg.data = "System is awake."
    rospy.loginfo(f"Published wakeup message: {wakeup_msg.data}")
    pub_wakeup.publish(wakeup_msg)  # 发布消息

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node("talker", anonymous=True)
    # 创建发布者，发布到名为 'chatter' 的话题
    pub_stop = rospy.Publisher("stop", Bool, queue_size=10)
    pub_error = rospy.Publisher("YHS_CIR02/Error", String, queue_size=10)
    pub_move = rospy.Publisher("lightcon/movestatus", UInt8, queue_size=10)
    pub_battery = rospy.Publisher("battery_pub", Battery, queue_size=10)
    pub_wakeup = rospy.Publisher("aikit/wakenUp", String, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # 创建要发布的消息
        # 获取输入
        user_input = input("1.stop,2.Error,3.movestatus,4.battery_pub,5.wakenUp,6.all\n")

        if user_input == "1":
            stop()
        elif user_input == "2":
            error()
        elif user_input == "3":
            move()
        elif user_input == "4":
            battery()
        elif user_input == "5":
            wakeup()
        elif user_input =="6":
            stop()
            error()
            move()
            battery()
            wakeup()
        else:
            rospy.logwarn("Invalid input. Please enter a number between 1 and 6.")
        rate.sleep()  # 按照设定的速率休眠
