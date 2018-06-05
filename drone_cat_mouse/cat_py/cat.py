#!/usr/bin/python3
#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Alberto Martin Florido <almartinflorido@gmail.com>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#

import sys
import config
import comm
from MyAlgorithm import MyAlgorithm
sys.path.append('/usr/lib/python2.7/')
import types

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)

class Cat():
    def __init__(self):
    
        cfg = config.load('cat_conf.yml')

        #starting comm
        jdrc= comm.init(cfg, 'Introrob')

        self.camera = jdrc.getCameraClient("Introrob.Camera")
        self.navdata = jdrc.getNavdataClient("Introrob.Navdata")
        self.pose = jdrc.getPose3dClient("Introrob.Pose3D")
        self.cmdvel = jdrc.getCMDVelClient("Introrob.CMDVel")
        self.extra = jdrc.getArDroneExtraClient("Introrob.Extra")

        self.algorithm=MyAlgorithm(self.camera, self.navdata, self.pose, self.cmdvel, self.extra)
        print "Cat Components initialized OK"

    def play(self):
        # self.unpause_physics_client()
        self.algorithm.play()
        print "Cat is running"
        
    def pause (self):
        self.algorithm.pause()
        #self.pause_physics_client()
        print "Cat has been paused"
         
    def stop(self):
        self.algorithm.stop()
        print "Cat stopped"
                   
    def setExecute(self, execute):
        self.algorithm.algorithm = types.MethodType(execute, self.algorithm )
        print "Code updated"       

    def move(self):
        self.algorithm.move()
    
    def get_threshold_image(self):
        return self.algorithm.get_threshold_image()
    
    def get_color_image(self):
        return self.algorithm.get_color_image()