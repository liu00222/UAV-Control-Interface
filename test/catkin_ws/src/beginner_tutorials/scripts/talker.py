#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

import socket

pose = PoseStamped()


def make_pose_stamped(orientation, position):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()

    msg.pose.orientation.x = orientation[0]
    msg.pose.orientation.y = orientation[1]
    msg.pose.orientation.z = orientation[2]
    msg.pose.orientation.w = orientation[3]

    msg.pose.position.x = position[0]  # red
    msg.pose.position.y = position[1]  # green
    msg.pose.position.z = position[2]  # blue

    return msg


def callback(data):
    global pose
    pose = data


def talker():
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=20)

    rospy.init_node('talker', anonymous=True)

    host = '10.10.21.104'
    port = 3000
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))

    num_of_robots = 1

    rospy.Subscriber('/firefly/ground_truth/pose', PoseStamped, callback)

    land_on = False

    while not rospy.is_shutdown():
        position = pose.pose.position
        x, y, z = position.x, position.y, position.z

        data, addr = s.recvfrom(1024)
        data = data.decode('utf-8')
        rospy.loginfo(data)

        if data == "connection request from the client side":
            s.sendto((str(num_of_robots)).encode('utf-8'), addr)
            s.sendto((str(x) + ',' + str(y) + ',' + str(z)).encode('utf-8'), addr)

        if data == "forward" and not land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x + 1.0, y, z])
            pub.publish(msg)

        elif data == "backward" and not land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x - 1.0, y, z])
            pub.publish(msg)

        elif data == "upward" and not land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y, z + 1.0])
            pub.publish(msg)

        elif data == "downward" and not land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y, z - 1.0])
            pub.publish(msg)

        elif data == "leftward" and not land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y + 1.0, z])
            pub.publish(msg)

        elif data == "rightward" and not land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y - 1.0, z])
            pub.publish(msg)

        elif data == "land on" and not land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y, 0.0])
            pub.publish(msg)
            land_on = True

        elif data == "take off" and land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y, 1.0])
            pub.publish(msg)
            land_on = False

        s.sendto("none".encode('utf-8'), addr)
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
