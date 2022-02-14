#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class InitiateRescue:

  def __init__(self):
    rospy.init_node('initiate_rescue')
    self.pub = rospy.Publisher('/rescue_items', String, queue_size=10)

  def run(self):
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        items = []
        for _ in range(3):
            item = input("Add item to rescue: ")
            items.append(item)
        msg = ' '.join(items)
        print(f'Rescuing {msg}')
        self.pub.publish(msg)

if __name__ == '__main__':
    node = InitiateRescue()
    node.run()
