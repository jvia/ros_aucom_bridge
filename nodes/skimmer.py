#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_aucom_bridge')
import rospy
from geometry_msgs.msg import Twist
from ros_aucom_bridge.msg import TopicSignature

pub = rospy.Publisher('skimmed_topics', TopicSignature)

def skimmer():
    rospy.init_node('skimmer')
    # get topics from list parameter
    if not rospy.has_param('~topics'):
        rospy.logerror('No topics list found. Ensure the topics parameter has a string of topics.')
        exit(1)

    topics = rospy.get_param("~topics")
    for line in topics.splitlines():
        topic_name, topic_type = line.split()
        rospy.loginfo('Topic name: %s, Topic type: %s', topic_name, topic_type)
        di(topic_type)
        class_name = topic_type.split('/')[1]

        print class_name
        rospy.Subscriber(topic_name,
                         eval(class_name),
                         make_subsriber(topic_name, topic_type))
        rospy.spin()


def di(topic_type):
    msg = topic_type.partition('/')[0] + '.msg'
    try:
        rospy.loginfo('import %s', msg)
        __import__(msg)
    except ImportError:
        rospy.logerr('Could not import %s. Trying without .msg', msg)
        try:
            msg = msg.partition('.')[0]
            __import__(msg)
        except ImportError:
            rospy.logerr('Could not import %s. Skipping', msg)

def make_subsriber(topic_name, topic_type):
    return lambda msg: topic_publisher(msg, topic_name, topic_type)


def topic_publisher(msg, topic_name, topic_publisher):
    pub.publish(rospy.get_rostime(), topic_name, topic_type)
    rospy.loginfo('%s', '{0:10} :: {1:<15} {2:<15}'.format(rospy.get_rostime(), topic_name, topic_type))


def rostime_to_ms():
    return (rospy.get_rostime().secs - time.secs) * 1000 + (rospy.get_rostime().nsecs - time.nsecs) / 1000000

def ignore(topic_type):
    for ignored in ignore_list:
        if (ignored == topic_type): return True
    return False

if __name__ == '__main__':
    try:
        skimmer()
    except rospy.ROSInterruptException: pass
