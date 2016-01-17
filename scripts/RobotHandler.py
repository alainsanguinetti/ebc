#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
This file defines the robot handler class.
It is a representation of a robot that provides method to register a robot
"""

import roslaunch
import rospy
from ebc.msg import Event

class RobotHandler ( object ):
    """
    The robot handler class
    """
    
    def __init__ ( self, r_name, launch, target ):
        """
        Creates the robot node if it doesnot exist yet
        """
        package = "ebc"
        executable = "Robot.py"
        self.launch = launch
       
        self.r_name = r_name
        
        self.node = roslaunch.core.Node(package, executable, name=r_name, output="screen" )
        self.process = self.launch.launch( self.node )
        
        # Register the subscriber for event
        self.sub = rospy.Subscriber( self.r_name, Event, target.robot_callback )
        
        # Create publisher for task event TODO
            
            
            
    def stop ( self ):
        """
        Stop the robot process
        """
        self.process.stop()
        
    
