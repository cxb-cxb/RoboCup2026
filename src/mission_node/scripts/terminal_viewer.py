#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import threading

current_topic = '/aft_mapped_to_init'
subscriber = None
lock = threading.Lock()

def odom_callback(msg):
    pos = msg.pose.pose.position
    print(f"x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f}")

def subscribe_topic(topic_name):
    global subscriber
    with lock:
        if subscriber is not None:
            subscriber.unregister()
        subscriber = rospy.Subscriber(topic_name, Odometry, odom_callback)
        print(f"已切换到话题: {topic_name}")

def input_thread():
    global current_topic
    while not rospy.is_shutdown():
        new_topic = input("输入新的话题名切换（当前: {}，回车保持不变）：".format(current_topic)).strip()
        if new_topic and new_topic != current_topic:
            current_topic = new_topic
            subscribe_topic(current_topic)

if __name__ == '__main__':
    rospy.init_node('terminal_viewer', anonymous=True)
    subscribe_topic(current_topic)
    t = threading.Thread(target=input_thread)
    t.daemon = True
    t.start()
    rospy.spin()
