from __future__ import print_function
import rospy
import actionlib
import uav_motion.msg
import geometry_msgs.msg
import numpy as np
from std_srvs.srv import Empty

# 全局变量
waypoints = []
current_index = 0
client = None


def send_waypoint(event):
    global current_index
    if current_index >= len(waypoints):
        rospy.loginfo("All waypoints sent.")
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


if __name__ == '__main__':
    rospy.init_node('waypoints_client', anonymous=False)
    rospy.wait_for_service('stop_sampling')
    stop_srv_client_ = rospy.ServiceProxy('stop_sampling', Empty)

    # 初始化 action client
    client = actionlib.SimpleActionClient(
        'waypoints', uav_motion.msg.waypointsAction)
    client.wait_for_server()

    # 读取路径点
    waypoints = read_waypoints_from_file(
        '/home/ubuntu/uav_motion_ws/src/uav_motion/launch/waypoints.txt')

    if not waypoints:
        rospy.logerr("No valid waypoints loaded.")
        exit(1)

    # 设定定时器触发时间
    start_time = rospy.Time.now()
    for i, wp in enumerate(waypoints):
        delay = rospy.Duration(wp[4])  # 最后一项是发送时间
        rospy.Timer(delay, send_waypoint, oneshot=True)

    rospy.spin()
