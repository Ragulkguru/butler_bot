#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CafeRobot:
    def __init__(self):
        rospy.init_node('cafe_robot_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.order_sub = rospy.Subscriber('/order', String, self.order_callback)
        self.current_order = None
        self.home_position = [0, 0]
        self.kitchen_position = [1, 0]
        self.table_positions = {
            "table1": [2, 1],
            "table2": [2, 2],
            "table3": [2, 3]
        }

    def order_callback(self, msg):
        self.current_order = msg.data
        self.process_order()

    def process_order(self):
        if self.current_order:
            destination = self.kitchen_position
            self.navigate_to(destination)

            if self.wait_for_confirmation("kitchen"):
                table = self.table_positions[self.current_order]
                self.navigate_to(table)

                if not self.wait_for_confirmation("table"):
                    self.navigate_to(self.kitchen_position)

            self.navigate_to(self.home_position)
            self.current_order = None

    def navigate_to(self, position):
        twist = Twist()
        twist.linear.x = 0.5
        duration = self.calculate_travel_time(position)
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(duration)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def calculate_travel_time(self, position):
        current_position = self.get_current_position()
        distance = ((position[0] - current_position[0]) ** 2 + (position[1] - current_position[1]) ** 2) ** 0.5
        return distance / 0.5

    def get_current_position(self):
        # This is a placeholder; implement with real odometry data
        return self.home_position

    def wait_for_confirmation(self, location):
        # This is a placeholder; implement with real confirmation logic
        rospy.sleep(2)
        return True

if __name__ == '__main__':
    try:
        robot = CafeRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
