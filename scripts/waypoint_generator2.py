from __future__ import print_function
import rospy
import actionlib
import uav_motion.msg
import geometry_msgs.msg
import numpy as np
from std_srvs.srv import Empty
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

# 全局变量
waypoints = []
current_index = 0
client = None
timers_started = False  # 防止重复启动定时器
completedPub = None

def send_waypoint(event):
    global current_index
    # if current_index >= len(waypoints):
    #     rospy.loginfo("All waypoints sent. Shutting down node.")
    #     rospy.signal_shutdown("Waypoint mission completed")
    #     return

    # 构造 goal 并发送
    goal = uav_motion.msg.waypointsGoal()
    wp = waypoints[current_index]
    
    pos = geometry_msgs.msg.Point(wp[0], wp[1], wp[2])
    goal.positions.append(pos)
    goal.yaws.append(wp[3])
    # trial
    goal.durations.append(wp[5])

    client.send_goal(goal)
    rospy.loginfo(f"Sent waypoint {current_index + 1}: {wp}")
    client.wait_for_result()
    rospy.loginfo(f"Result: {client.get_result()}")

    current_index += 1

def send_end_signal(event):
    rospy.loginfo("All waypoints sent.")
    msg = Bool()
    msg.data = True  # 表示任务完成
    rospy.loginfo("Publishing: mission completed = True")
    completedPub.publish(msg)
    return

def read_waypoints_from_file(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            values = [float(x.strip()) for x in line.strip().split(',')]
            if len(values) != 6:
                rospy.logwarn(f"Skipping invalid line: {line}")
                continue
            data.append(values)
    return data

def flight_state_callback(msg):
    global timers_started

    if msg.data == 3 and not timers_started:
        rospy.loginfo("TRACKING mode detected. Starting waypoint timers.")
        timers_started = True
        start_waypoint_timers()

def start_waypoint_timers():
    for i, wp in enumerate(waypoints):
        delay = rospy.Duration(wp[4])
        rospy.Timer(delay, send_waypoint, oneshot=True)
        if i == len(waypoints) - 1: # extra timer
            delay = rospy.Duration(wp[4] + wp[5])
            rospy.Timer(delay, send_end_signal, oneshot=True) 
        

if __name__ == '__main__':
    rospy.init_node('waypoints_client', anonymous=False)
    
    rospy.wait_for_service('stop_sampling')
    stop_srv_client_ = rospy.ServiceProxy('stop_sampling', Empty)

    # 初始化 action client
    client = actionlib.SimpleActionClient('waypoints', uav_motion.msg.waypointsAction)
    client.wait_for_server()

    completedPub = rospy.Publisher('/tracking_completed', Bool, queue_size=10, latch=True)

    # 读取路径点
    waypoints = read_waypoints_from_file(
        '/home/ubuntu/uav_motion_ws/src/uav_motion/launch/waypoints.txt')

    if not waypoints:
        rospy.logerr("No valid waypoints loaded.")
        exit(1)

    # 订阅 MAVROS 状态
    # rospy.Subscriber("/mavros/state", State, state_callback)
    rospy.Subscriber('/flight_state', UInt8, flight_state_callback)

    rospy.spin()