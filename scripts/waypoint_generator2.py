from __future__ import print_function
import rospy
import actionlib
import uav_motion.msg
import geometry_msgs.msg
import numpy as np
from std_srvs.srv import Empty
from mavros_msgs.msg import State

# 全局变量
waypoints = []
current_index = 0
client = None
timers_started = False  # 防止重复启动定时器

def send_waypoint(event):
    global current_index
    if current_index >= len(waypoints):
        rospy.loginfo("All waypoints sent. Shutting down node.")
        rospy.signal_shutdown("Waypoint mission completed")
        return

    # 构造 goal 并发送
    goal = uav_motion.msg.waypointsGoal()
    wp = waypoints[current_index]
    
    pos = geometry_msgs.msg.Point(wp[0], wp[1], wp[2])
    goal.positions.append(pos)
    goal.yaws.append(wp[3])

    client.send_goal(goal)
    rospy.loginfo(f"Sent waypoint {current_index + 1}: {wp}")
    client.wait_for_result()
    rospy.loginfo(f"Result: {client.get_result()}")

    current_index += 1

def read_waypoints_from_file(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            values = [float(x.strip()) for x in line.strip().split(',')]
            if len(values) != 5:
                rospy.logwarn(f"Skipping invalid line: {line}")
                continue
            data.append(values)
    return data

def state_callback(msg):
    global timers_started

    if msg.mode == "OFFBOARD" and not timers_started:
        rospy.loginfo("OFFBOARD mode detected. Starting waypoint timers.")
        timers_started = True
        start_waypoint_timers()

def start_waypoint_timers():
    for i, wp in enumerate(waypoints):
        delay = rospy.Duration(wp[4])
        rospy.Timer(delay, send_waypoint, oneshot=True)

if __name__ == '__main__':
    rospy.init_node('waypoints_client', anonymous=False)
    
    rospy.wait_for_service('stop_sampling')
    stop_srv_client_ = rospy.ServiceProxy('stop_sampling', Empty)

    # 初始化 action client
    client = actionlib.SimpleActionClient('waypoints', uav_motion.msg.waypointsAction)
    client.wait_for_server()

    # 读取路径点
    waypoints = read_waypoints_from_file(
        '/home/ubuntu/uav_motion_ws/src/uav_motion/launch/waypoints.txt')

    if not waypoints:
        rospy.logerr("No valid waypoints loaded.")
        exit(1)

    # 订阅 MAVROS 状态
    rospy.Subscriber("/mavros/state", State, state_callback)

    rospy.spin()