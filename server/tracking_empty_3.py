#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

import socket

pose1 = PoseStamped()
pose2 = PoseStamped()
pose3 = PoseStamped()
pose4 = PoseStamped()
pose5 = PoseStamped()


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


class UAV:
    def __init__(self, x, y, z, identity):
        self.land_on = False
        self.x, self.y, self.z = x, y, z
        self.id = identity
        self.pose_publisher = rospy.Publisher(('/firefly' + str(self.id + 1) + '/command/pose'),
                                              PoseStamped, queue_size=20)


target = UAV(2.0, -2.0, 1.0, 5)
is_tracking = False
target_direction = 1


class Server:
    def __init__(self):
        rospy.init_node('touch_interface_swarm_server', anonymous=True)

        self.host = '192.168.1.107'
        self.port = 3000
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((self.host, self.port))

        self.interface_address = None

        self.uav0 = UAV(0.0, -1.0, 1.0, 0)
        self.uav1 = UAV(0.0, 0.0, 1.0, 1)

        self.num_of_robots = 2

        self.d = 1
        self.h = 1
        self.v = 1
        self.formation = "None"

        self.tracking_distance = 0.0

        rospy.Subscriber('/firefly1/ground_truth/pose', PoseStamped, self.get_position1)
        rospy.Subscriber('/firefly2/ground_truth/pose', PoseStamped, self.get_position2)
        # rospy.Subscriber('/firefly3/ground_truth/pose', PoseStamped, get_position3)
        # rospy.Subscriber('/firefly4/ground_truth/pose', PoseStamped, get_position4)
        # rospy.Subscriber('/firefly5/ground_truth/pose', PoseStamped, get_position5)
        rospy.Subscriber('/firefly6/ground_truth/pose', PoseStamped, self.move_target)

    def make_str(self, id, x, y, z):
        return (str(id) + '&' + str(x) + '&' + str(y) + '&' + str(z))

    def get_position1(self, data):
        if self.interface_address is not None:
            global pose1
            pose1 = data
            x = data.pose.position.x
            y = data.pose.position.y
            z = data.pose.position.z

            self.s.sendto((self.make_str(0, x, y, z)).encode('utf-8'), self.interface_address)

    def get_position2(self, data):
        if self.interface_address is not None:
            global pose2
            pose2 = data
            x = data.pose.position.x
            y = data.pose.position.y
            z = data.pose.position.z

            self.s.sendto((self.make_str(1, x, y, z)).encode('utf-8'), self.interface_address)

    def move_target(self, data):
        if self.interface_address is not None:
            global pose1
            pose1 = data
            x = data.pose.position.x
            y = data.pose.position.y
            z = data.pose.position.z

            self.s.sendto((self.make_str('t', x, y, z)).encode('utf-8'), self.interface_address)

        position = data.pose.position
        global target
        global target_direction

        # target is at (2, -2, 1)
        if abs(position.x - 2.0) < 0.5 and abs(position.y + 2.0) < 0.5:
            target_direction = 1
            target.x = 2.0
            target.y = 2.0
            target.z = 1.0
            msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [target.x, target.y, target.z])
            target.pose_publisher.publish(msg0)

        elif abs(position.x - 2.0) < 0.5 and abs(position.y - 2.0) < 0.5:
            target_direction = 2
            target.x = -2.0
            target.y = 2.0
            target.z = 1.0
            msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [target.x, target.y, target.z])
            target.pose_publisher.publish(msg0)

        elif abs(position.x + 2.0) < 0.5 and abs(position.y - 2.0) < 0.5:
            target_direction = 3
            target.x = -2.0
            target.y = -2.0
            target.z = 1.0
            msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [target.x, target.y, target.z])
            target.pose_publisher.publish(msg0)

        elif abs(position.x + 2.0) < 0.5 and abs(position.y + 2.0) < 0.5:
            target_direction = 4
            target.x = 2.0
            target.y = -2.0
            target.z = 1.0
            msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [target.x, target.y, target.z])
            target.pose_publisher.publish(msg0)

        global is_tracking
        if is_tracking:
            # to 1st quadrant
            if target_direction == 1:
                self.uav0.x = target.x - 0.5 - self.tracking_distance
                self.uav0.y = target.y - self.tracking_distance

                self.uav1.x = target.x + 0.5 + self.tracking_distance
                self.uav1.y = target.y - self.tracking_distance

            # to 2nd quadrant
            if target_direction == 2:
                self.uav0.x = target.x + self.tracking_distance
                self.uav0.y = target.y - 0.5 - self.tracking_distance

                self.uav1.x = target.x + self.tracking_distance
                self.uav1.y = target.y + 0.5 + self.tracking_distance

            # to 3rd quadrant
            if target_direction == 3:
                self.uav0.x = target.x + 0.5 + self.tracking_distance
                self.uav0.y = target.y + self.tracking_distance

                self.uav1.x = target.x - 0.5 - self.tracking_distance
                self.uav1.y = target.y + self.tracking_distance

            # to 4th quadrant
            if target_direction == 4:
                self.uav0.x = target.x - self.tracking_distance
                self.uav0.y = target.y + 0.5 + self.tracking_distance

                self.uav1.x = target.x - self.tracking_distance
                self.uav1.y = target.y - 0.5 - self.tracking_distance

            msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
            self.uav0.pose_publisher.publish(msg0)

            msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
            self.uav1.pose_publisher.publish(msg0)

    def command_formation(self):
        self.formation = "Triangle"

        self.uav0.x = 0.0
        self.uav0.y = -1.0 * self.d
        self.uav0.z = 1.0
        msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
        self.uav0.pose_publisher.publish(msg0)

        self.uav1.x = 1.0 * self.d
        self.uav1.y = 0.0
        self.uav1.z = 1.0
        msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
        self.uav1.pose_publisher.publish(msg1)

    def command_horizontal(self):
        self.formation = "horizontal"

        self.uav0.x = 0.0
        self.uav0.y = -1.0 * self.h
        self.uav0.z = 1.0
        msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
        self.uav0.pose_publisher.publish(msg0)

        self.uav1.x = 0.0
        self.uav1.y = 0.0
        self.uav1.z = 1.0
        msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
        self.uav1.pose_publisher.publish(msg1)

    def command_vertical(self):
        self.formation = "vertical"

        self.uav0.x = -1.0 * self.v
        self.uav0.y = 0.0
        self.uav0.z = 1.0
        msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
        self.uav0.pose_publisher.publish(msg0)

        self.uav1.x = 0.0
        self.uav1.y = 0.0
        self.uav1.z = 1.0
        msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
        self.uav1.pose_publisher.publish(msg1)

    def command_spread_out(self):
        if self.formation == "Triangle":
            self.d = self.d * 1.2
            self.command_formation()

        elif self.formation == "horizontal":
            self.h = self.h * 1.2
            self.command_horizontal()

        elif self.formation == "vertical":
            self.v = self.v * 1.2
            self.command_vertical()

    def command_aggregation(self):
        if self.formation == "Triangle":
            self.d = self.d * 0.8
            self.command_formation()

        elif self.formation == "horizontal":
            self.h = self.h * 0.8
            self.command_horizontal()

        elif self.formation == "vertical":
            self.v = self.v * 0.8
            self.command_vertical()

    def command_forward(self):
        self.uav0.x += 1.0
        msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
        self.uav0.pose_publisher.publish(msg0)

        self.uav1.x += 1.0
        msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
        self.uav1.pose_publisher.publish(msg1)

    def command_backward(self):
        self.uav0.x -= 1.0
        msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
        self.uav0.pose_publisher.publish(msg0)

        self.uav1.x -= 1.0
        msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
        self.uav1.pose_publisher.publish(msg1)

    def command_upward(self):
        self.uav0.z += 1.0
        msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
        self.uav0.pose_publisher.publish(msg0)

        self.uav1.z += 1.0
        msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
        self.uav1.pose_publisher.publish(msg1)

    def command_downward(self):
        if self.uav0.z > 0.0:
            self.uav0.z -= 1.0
            msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
            self.uav0.pose_publisher.publish(msg0)

            self.uav1.z -= 1.0
            msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
            self.uav1.pose_publisher.publish(msg1)

    def command_leftward(self):
        self.uav0.y += 1.0
        msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
        self.uav0.pose_publisher.publish(msg0)

        self.uav1.y += 1.0
        msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
        self.uav1.pose_publisher.publish(msg1)

    def command_rightward(self):
        self.uav0.y -= 1.0
        msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
        self.uav0.pose_publisher.publish(msg0)

        self.uav1.y -= 1.0
        msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
        self.uav1.pose_publisher.publish(msg1)

    def process_data(self, data, address):
        #
        # Temporary mapping:
        # specific speed -> formation
        # increase speed -> spread out
        # decrease speed -> aggregation
        #
        if data == "c":
            self.s.sendto((str(self.num_of_robots)).encode('utf-8'), address)
            self.s.sendto("tracking_empty_3".encode('utf-8'), address)

        elif data == "specific speed":
            self.d = 1.0
            self.command_formation()

        elif data == "increase speed":
            self.command_spread_out()
            #global is_tracking
            #is_tracking = True

        elif data == "decrease speed":
            self.command_aggregation()
            #global is_tracking
            #is_tracking = False

        elif data == "tracking":
            global is_tracking
            is_tracking = not is_tracking

        elif data == "forward" and not self.uav0.land_on:
            self.command_forward()

        elif data == "backward" and not self.uav0.land_on:
            self.command_backward()

        elif data == "upward" and not self.uav0.land_on:
            self.command_upward()

        elif data == "downward" and not self.uav0.land_on:
            self.command_downward()

        elif data == "leftward" and not self.uav0.land_on:
            self.command_leftward()

        elif data == "rightward" and not self.uav0.land_on:
            self.command_rightward()

        elif data == "land on" and not self.uav0.land_on:
            self.uav0.z = 0.0
            msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
            self.uav0.pose_publisher.publish(msg0)

            self.uav1.z = 0.0
            msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
            self.uav1.pose_publisher.publish(msg1)

            self.uav0.land_on = True
            self.uav1.land_on = True

        elif data == "take off" and self.uav0.land_on:
            self.uav0.z = 1.0
            msg0 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav0.x, self.uav0.y, self.uav0.z])
            self.uav0.pose_publisher.publish(msg0)

            self.uav1.z = 1.0
            msg1 = make_pose_stamped([0.0, 0.0, 0.0, 1.0], [self.uav1.x, self.uav1.y, self.uav1.z])
            self.uav1.pose_publisher.publish(msg1)

            self.uav0.land_on = False
            self.uav1.land_on = False

        elif data == "horizontal" and not self.uav0.land_on:
            # self.command_horizontal()
            self.tracking_distance -= 0.75

        elif data == "vertical" and not self.uav0.land_on:
            # self.command_vertical()
            self.tracking_distance += 0.75

    def print_starting_msg(self):
        print("\n -------------------------------------")
        print("| AI Touch Interface Swarm Server Started")
        print("|")
        print("| IP Address: " + str(self.host))
        print("| Port: " + str(self.port))
        print(" -------------------------------------\n")

    def run(self):
        self.print_starting_msg()

        while not rospy.is_shutdown():
            data, address = self.s.recvfrom(1024)
            data = data.decode('utf-8')
            self.interface_address = address

            rospy.loginfo(data)

            self.process_data(data, address)

            # self.s.sendto("none".encode('utf-8'), address)


if __name__ == '__main__':
    try:
        Server().run()
    except rospy.ROSInterruptException:
        pass
