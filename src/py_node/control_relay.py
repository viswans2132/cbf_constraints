#!/usr/bin/env python3

import rospy

from collections import deque
from geometry_msgs.msg import Twist, TwistStamped


class VarTopicRelay:
    def __init__(self, in_topic, out_topic, msg_type, start_time):
        self.in_topic = in_topic
        self.out_topic = out_topic
        self.msg_type = msg_type
        self.start_time = start_time

        self.buffer = deque()
        self.prev_delay = None

        self.pub = rospy.Publisher(out_topic, msg_type, queue_size=10)
        self.sub = rospy.Subscriber(in_topic, msg_type, self.callback, queue_size=100)

    def get_delay(self):
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        block = int(elapsed // 30)
        delay = 0.0 if block % 2 == 0 else 0.07

        if delay != self.prev_delay:
            rospy.loginfo(f"[{self.out_topic}] Delay switched to {delay:.3f} s")
            self.prev_delay = delay

        return delay

    def callback(self, msg):
        now = rospy.Time.now()
        delay = self.get_delay()

        # Store each message with the delay active at arrival time
        self.buffer.append((msg, now, delay))

        # Optional: prevent unlimited growth
        while len(self.buffer) > 200:
            self.buffer.popleft()

    def try_publish(self, now):
        if not self.buffer:
            return

        # Find all messages that have completed their delay
        ready = []
        while self.buffer:
            msg, stamp, delay = self.buffer[0]
            if now >= stamp + rospy.Duration.from_sec(delay):
                ready.append(self.buffer.popleft())
            else:
                break

        if ready:
            # Publish only the newest ready message, discard older ready ones
            msg, stamp, delay = ready[-1]
            self.pub.publish(msg)

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

        delay = 0.08

        for i in range(1, self.num_agents + 1):

            drone_in = f"/dcf{i}/vel_msg1"
            drone_out = f"/dcf{i}/vel_msg"

            ugv_in = f"/demo_turtle{i}/cmd_vel1"
            ugv_out = f"/demo_turtle{i}/cmd_vel"

            self.relays.append(
                TopicRelay(drone_in, drone_out, TwistStamped, delay)
            )

            self.relays.append(
                TopicRelay(ugv_in, ugv_out, Twist, delay)
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