#!/usr/bin/env python3

import rospy

# Replace with your actual packages
from cf_cbf.msg import DroneConstraintMsg
from tb_cbf.msg import UgvConstraintMsg
from collections import deque


class VarTopicRelay:

    def __init__(self, in_topic, out_topic, msg_type, start_time):
        self.in_topic = in_topic
        self.out_topic = out_topic
        self.msg_type = msg_type
        self.start_time = start_time

        self.latest_msg = None
        self.release_time = None

        self.pub = rospy.Publisher(out_topic, msg_type, queue_size=10)
        self.sub = rospy.Subscriber(in_topic, msg_type, self.callback, queue_size=100)

    def get_delay(self):
        """Alternate delay every 20 seconds"""
        elapsed = (rospy.Time.now() - self.start_time).to_sec()

        block = int(elapsed // 20)

        if block % 2 == 0:
            return 0.0
        else:
            return 0.01

    def callback(self, msg):
        delay = self.get_delay()

        self.latest_msg = msg
        self.release_time = rospy.Time.now() + rospy.Duration.from_sec(delay)

    def try_publish(self, now):
        if self.latest_msg is not None and self.release_time is not None:
            if now >= self.release_time:
                self.pub.publish(self.latest_msg)
                self.latest_msg = None
                self.release_time = None




class TopicRelay:

    def __init__(self, in_topic, out_topic, msg_type, delay):
        self.in_topic = in_topic
        self.out_topic = out_topic
        self.msg_type = msg_type
        self.delay = rospy.Duration.from_sec(delay)

        self.buffer = deque()

        self.pub = rospy.Publisher(out_topic, msg_type, queue_size=10)
        self.sub = rospy.Subscriber(in_topic, msg_type, self.callback, queue_size=100)

    def callback(self, msg):
        now = rospy.Time.now()
        self.buffer.append((msg, now))

    def try_publish(self, now):

        while self.buffer:

            msg, stamp = self.buffer[0]

            if now - stamp >= self.delay:
                self.pub.publish(msg)
                self.buffer.popleft()
            else:
                break





class MultiAgentConstraintRelay:

    def __init__(self):
        self.num_agents = rospy.get_param("~no_of_agents", 3)
        self.loop_rate = rospy.get_param("~loop_rate", 60.0)

        self.start_time = rospy.Time.now()
        self.relays = []

        delay = 0.0

        for i in range(1, self.num_agents + 1):

            drone_in = f"/dcf{i}/cons1"
            drone_out = f"/dcf{i}/cons"

            ugv_in = f"/demo_turtle{i}/cons1"
            ugv_out = f"/demo_turtle{i}/cons"

            self.relays.append(
                TopicRelay(drone_in, drone_out, DroneConstraintMsg, delay)
            )

            self.relays.append(
                TopicRelay(ugv_in, ugv_out, UgvConstraintMsg, delay)
            )

            rospy.loginfo(f"Relay: {drone_in} → {drone_out}")
            rospy.loginfo(f"Relay: {ugv_in} → {ugv_out}")

    def run(self):
        rate = rospy.Rate(self.loop_rate)

        while not rospy.is_shutdown():

            now = rospy.Time.now()

            for relay in self.relays:
                relay.try_publish(now)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("multi_agent_constraint_relay")

    node = MultiAgentConstraintRelay()
    node.run()