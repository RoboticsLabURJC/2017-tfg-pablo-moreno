import rospy
from mavros_msgs.srv import CommandBool,CommandTOL, SetMode
import threading
import time

class PublisherExtra:
    '''
        ROS CMDVel Publisher. CMDVel Client to Send CMDVel to ROS nodes.
    '''
    def __init__(self, topicArming, topicLand, topicSetMode, topicTakeOff):
        '''
        PublisherCMDVel Constructor.

        @param topic: ROS topic to publish

        @type topic: String

        '''
        self.topicArming = topicArming
        self.topicLand = topicLand
        self.topicSetMode = topicSetMode

        self.lock = threading.Lock()


        self.arming_client = rospy.ServiceProxy(topicArming,CommandBool)
        self.land_client = rospy.ServiceProxy(topicLand,CommandTOL)
        self.takeoff_client = rospy.ServiceProxy(topicTakeOff,CommandTOL)
        self.set_mode_client = rospy.ServiceProxy(topicSetMode,SetMode)

    def arming(self):
        self.lock.acquire()
        print("Arming...")
        self.arming_client.call(value=True)
        time.sleep(2)
        print("Arming Done")
        self.lock.release()

    def land(self):
        self.lock.acquire()
        print("Landing...")
        self.land_client.call(0,0,0,0,0)
        print("Land")
        self.lock.release()

    def toggleCam(self):
        pass

    def change_mode(self):
        self.lock.acquire()
        self.set_mode_client.call(custom_mode="OFFBOARD")
        print("Mode changed to: OFFBOARD")
        self.lock.release()

    def takeoff(self):
        self.lock.acquire()
        print("Taking Off...")
        self.takeoff_client.call(altitude=12.5, latitude=47.3977419, longitude=8.5457011, min_pitch=0, yaw=0)
        #time.sleep(2.0)
        print("TakeOff Done")
        self.lock.release()

    def reset(self):
        pass

    def record(self,record):
        pass
