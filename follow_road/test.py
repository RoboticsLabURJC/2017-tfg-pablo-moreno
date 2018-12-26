import rospy
import mavros
import thread
import threading
import time

from mavros import setpoint as SP

class SetpointPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        # publisher for mavros/setpoint_position/local
            self.pub = SP.get_pub_position_local(queue_size=1)
            # subscriber for mavros/local_position/local
            self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.reached)
            try:
                thread.start_new_thread(self.navigate, ())
            except:
                print("Error: Unable to start thread")

            self.done = False
            self.done_evt = threading.Event()

    def reached(self, topic):
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) < 0.5

        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y) and \
           is_near('Z', topic.pose.position.z, self.z):
            self.done = True
            self.done_evt.set()

    def navigate(self):
        rate = rospy.Rate(10) # 10hz

        while True:
            msg = SP.PoseStamped(
                header=SP.Header(
                    frame_id='',
                    stamp=rospy.Time.now()),
            )
            msg.pose.position.x=0.0
            msg.pose.position.y=0.0
            msg.pose.position.z=3.0

            rospy.loginfo(msg)
            self.pub.publish(msg)

def setpoint_demo():
    rospy.init_node('setpoint_position_demo')
    mavros.set_namespace()  # initialize mavros module with default namespace
    rate = rospy.Rate(10)

    setpoint = SetpointPosition()

if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass
