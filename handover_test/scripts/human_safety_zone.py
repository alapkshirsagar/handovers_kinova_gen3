#!/usr/bin/env python

import rospy
import tf
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from config import HandoverConfig
from std_msgs.msg import String
class human_safety_zone:
    def __init__(self):
        self.config = HandoverConfig()
        self.handover_failure_subscriber = rospy.Subscriber("/timer_start_request", String, self.handover_failure_callback)
        self.tf_listener = tf.TransformListener()
        self.soundHandle = SoundClient()
        rospy.sleep(1.0)
        self.soundHandle.stopAll()

        looprate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                (trans_robot, rot_robot) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_gripper, rospy.Time(0))
                # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
                (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
                print(trans_human)
                self.check_human_safety_zone(trans_human)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'Could not find transform'
                continue

            looprate.sleep()

    def check_human_safety_zone(self,trans_human):
        print("here")
        if trans_human[0] > self.config.safety_zone_boundaries[0] and trans_human[0] < self.config.safety_zone_boundaries[1] and trans_human[1] > self.config.safety_zone_boundaries[2] and trans_human[1] < self.config.safety_zone_boundaries[3]:
            self.soundHandle.play(SoundRequest.NEEDS_PLUGGING)
            rospy.sleep(0.02)

    def handover_failure_callback(self, message):
        message_list = message.data.split(',')
        if message_list[0] == 'abort':
            self.soundHandle.play(SoundRequest.NEEDS_UNPLUGGING)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('human_safety_zone')
    try:
        check_safety = human_safety_zone()

    except rospy.ROSInterruptException:
        pass


       