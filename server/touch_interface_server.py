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


def get_position(data):
    global pose
    pose = data


class Server:
    def __init__(self):
        rospy.init_node('touch_interface_server', anonymous=True)

        # initialize the server
        self.host = '192.168.1.107'
        self.port = 3000
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((self.host, self.port))
        # end: initialize the server

        self.pose_publisher = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=20)
        self.num_of_robots = 1
        self.land_on = False

        self.x = 0.0
        self.y = 0.0
        self.z = 1.0

        rospy.Subscriber('/firefly/ground_truth/pose', PoseStamped, get_position)

    def process_data(self, data, x, y, z, address):
        if data == "c":
            self.s.sendto((str(self.num_of_robots)).encode('utf-8'), address)
            self.s.sendto((str(x) + ',' + str(y) + ',' + str(z)).encode('utf-8'), address)
            self.x, self.y, self.z = x, y, z

        if data == "forward" and not self.land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x + 1.0, y, z])
            self.x += 1.0
            self.pose_publisher.publish(msg)

        elif data == "backward" and not self.land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x - 1.0, y, z])
            self.x -= 1.0
            self.pose_publisher.publish(msg)

        elif data == "upward" and not self.land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y, z + 1.0])
            self.z += 1.0
            self.pose_publisher.publish(msg)

        elif data == "downward" and not self.land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y, z - 1.0])
            if self.z > 0.0:
                self.z -= 1.0
            self.pose_publisher.publish(msg)

        elif data == "leftward" and not self.land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y + 1.0, z])
            self.y += 1.0
            self.pose_publisher.publish(msg)

        elif data == "rightward" and not self.land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y - 1.0, z])
            self.y -= 1.0
            self.pose_publisher.publish(msg)

        elif data == "land on" and not self.land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y, 0.0])
            self.pose_publisher.publish(msg)
            self.z = 0.0
            self.land_on = True

        elif data == "take off" and self.land_on:
            msg = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [x, y, 1.0])
            self.pose_publisher.publish(msg)
            self.z = 1.0
            self.land_on = False

        elif data == "hover" and not self.land_on:
            self.pose_publisher.publish(make_pose_stamped([0.0, 0.0, 0.0, 1.0], [1.0, -1.0, 1.0]))
            self.x, self.y, self.z = 1.0, -1.0, 1.0
            rospy.sleep(2)

            self.pose_publisher.publish(make_pose_stamped([0.0, 0.0, 0.0, 1.0], [1.0, 1.0, 1.0]))
            self.x, self.y, self.z = 1.0, 1.0, 1.0
            rospy.sleep(2)

            self.pose_publisher.publish(make_pose_stamped([0.0, 0.0, 0.0, 1.0], [-1.0, 1.0, 1.0]))
            self.x, self.y, self.z = -1.0, 1.0, 1.0
            rospy.sleep(2)

            self.pose_publisher.publish(make_pose_stamped([0.0, 0.0, 0.0, 1.0], [-1.0, -1.0, 1.0]))
            self.x, self.y, self.z = -1.0, -1.0, 1.0
            rospy.sleep(2)

            self.pose_publisher.publish(make_pose_stamped([0.0, 0.0, 0.0, 1.0], [1.0, -1.0, 1.0]))
            self.x, self.y, self.z = 1.0, -1.0, 1.0

    def print_starting_msg(self):
        print("\n -------------------------------------")
        print("| AI Touch Interface Server Started")
        print("|")
        print("| IP Address: " + str(self.host))
        print("| Port: " + str(self.port))
        print(" -------------------------------------\n")

    def run(self):
        self.print_starting_msg()

        while not rospy.is_shutdown():

            # receive data from the ipad interface
            data, address = self.s.recvfrom(1024)
            data = data.decode('utf-8')

            # print the command from ipad
            if data != "none":
                rospy.loginfo(data)

            self.process_data(data, self.x, self.y, self.z, address)

            # send data back to the ipad
            self.s.sendto("none".encode('utf-8'), address)


if __name__ == '__main__':
    try:
        Server().run()
    except rospy.ROSInterruptException:
        pass
