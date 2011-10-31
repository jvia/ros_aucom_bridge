#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_aucom_bridge')
import rospy

from ros_aucom_bridge.msg import Signature

pub = rospy.Publisher('skimmed_topics', Signature)
ignore_list = ['rosgraph_msgs/Log', 'rosgraph_msgs/Clock', 'ros_aucom_bridge/Signature']
time = 0.0

def skimmer():
    rospy.init_node('skimmer')
    global time
    time = rospy.get_rostime()

    # get topics from list parameter
    if rospy.has_param('~topics'):
        print 'Reading in topics list'
        topics = rospy.get_param("~topics")
        for line in topics.splitlines():
            topic_name, topic_type = line.split()
            rospy.loginfo('Topic name: %s, Topic type: %s', topic_name, topic_type)
            di(topic_type)
    else:
        rospy.logerror('No topics list found, subscribing to all running topics. THIS WILL LIKELY NOT WORK')
        # for topic_name, topic_type in rospy.client.get_published_topics():
        #     if (not ignore(topic_type)):
        #         print 'Making callback for {0} of type {1}'.format(topic_name, topic_type.split('/')[1])
        # rospy.Subscriber(topic_name,
        #                  eval(class_name(topic_type)),
        #                  make_subsriber(topic_name, topic_type))
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

def dynamic_import(topic_type):
    print ''

def class_name(topic_type):
    return topic_type.split('/')[1]


def topic_publisher(msg, topic_name, topic_publisher):
    try:
        print '{0:10} :: {1:<15} {2:<15}'.format(rostime_to_ms(), topic_name, topic_publisher)
    except AttributeError:
        print 'ERROR: {0} {1}'.format(topic_name, topic_publisher)

def rostime_to_ms():
    return (rospy.get_rostime().secs - time.secs) * 1000 + (rospy.get_rostime().nsecs - time.nsecs) / 1000000
# return rospy.get_time() - time

def ignore(topic_type):
    """
    """
    for ignored in ignore_list:
        if (ignored == topic_type): return True
    return False

if __name__ == '__main__':
    try:
        skimmer()
    except rospy.ROSInterruptException: pass
