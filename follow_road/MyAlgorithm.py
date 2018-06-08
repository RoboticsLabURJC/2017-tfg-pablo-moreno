import threading
import time
from datetime import datetime
import cv2
import numpy as np
import math

from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient


time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        self.image = None

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def setImageFiltered(self, image):
        self.lock.acquire()
        self.image=image
        self.lock.release()

    def getImageFiltered(self):
        self.lock.acquire()
        tempImage=self.image
        self.lock.release()
        return tempImage

    def run (self):

        self.stop_event.clear()

        while (not self.kill_event.is_set()):

            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    def execute(self):
        # Add your code here
        input_image = self.camera.getImage().data

        if input_image is not None:
            image_HSV = cv2.cvtColor(input_image, cv2.COLOR_RGB2HSV)

            #Treshold image
            value_min_HSV = np.array([20, 0, 0])
            value_max_HSV = np.array([60, 130, 130])
            image_HSV_filtered = cv2.inRange(image_HSV, value_min_HSV, value_max_HSV)

            #Reducing noise
            closing = cv2.morphologyEx(image_HSV_filtered, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
            opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))

            #Filtered image
            image_HSV_filtered_Mask = np.dstack((opening, opening, opening))

            #drawing contours
            imgray = cv2.cvtColor(image_HSV_filtered_Mask, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(imgray, 127, 255, 0)
            _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image_HSV_filtered_Mask, contours, -1, (0,255,0), 3)

            #Getting the centre of the road
            area = []
            for pic, contour in enumerate(contours):
                area.append(cv2.contourArea(contour))
                print("Area: " + str(area))
            print("Size: " + str(len(area)))
            if len(area) > 1:
                if area[0] < area[1]:
                    M = cv2.moments(contours[1])
                else:
                    M = cv2.moments(contours[0])
            else:
                M = cv2.moments(contours[0])

            if int(M['m00']) != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print("cx: " + str(cx))
                print("cy: " + str(cy))

                #drawing the center
                cv2.circle(image_HSV_filtered_Mask, (cx, cy), 7, np.array([255, 0, 0]), -1)

                #move the drone
                if cx > 250:
                    self.cmdvel.sendCMDVel(0,0,0.5,0,0,0)
                    print("Elevating")
                elif cx < 20:
                    self.cmdvel.setVX(0.3)
                    self.cmdvel.setYaw(-0.1)
                    print("Detected two roads")

                else:
                    self.cmdvel.setVX(0.3)
                    self.cmdvel.setYaw((153-int(cx))*0.01)

                    print("Yaw: " + str((153-int(cx))*0.01))
                self.cmdvel.sendVelocities()
                
            #printing the filtered image
            self.setImageFiltered(image_HSV_filtered_Mask)

            '''
            If you want show a thresold image (black and white image)
            self.setImageFiltered(img)
            '''
