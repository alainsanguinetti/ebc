#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
This file defines the robot handler class.
It is a representation of a robot that provides method to register a robot
"""

import roslaunch
import rospy
from ebc.msg import Event
from ebc.msg import Task as TaskMsg
from Task import Task

class RobotHandler ( object ):
    """
    The robot handler class
    """
    
    def __init__ ( self, r_name, launch, target ):
        """
        Creates the robot node if it does not exist yet
        """
        package = "ebc"
        executable = "Robot.py"
        self.launch = launch

       
        self.r_name = r_name
        self.id = int(filter(str.isdigit, self.r_name))
        
        self.node = roslaunch.core.Node(package, executable, name=r_name, output="screen" )
        self.process = self.launch.launch( self.node )
        
        # Register the subscriber for event
        self.sub = rospy.Subscriber( self.r_name, Event, target.robot_callback )

        self.acceptTask = True
        self.task_id = None
        
        
        
    def assignTask( self, task, task_id ):
        """
        Stores a task in the robot handler
        """
        self.acceptTask = False
        self.task = task
        self.todo_index = 1
        self.task_id = task_id
        
        
        
    def nextSector( self ):
        """
        Return the id of the next sector for the robot
        """
        if self.task:
            return str(self.task.sectors[ self.todo_index + 1])
        else:
            return "home"
        
        
        
    def finishTask( self ):
        """
        Reset task related attributes
        """
        self.acceptTask = True
        self.task = None
        self.todo_index = None
        self.task_id = None
        
        
        
    def changeSector( self ):
        """
        Advance the index on the todo list by one step
        """
        # One more sector crossed
        self.todo_index += 1
        
        # check if this is the end of the task
        if self.todo_index == len( self.task.sectors ) - 1:
            # Free task
            self.finishTask()
            
            
            
    def stop ( self ):
        """
        Stop the robot process
        """
        self.process.stop()
        
    
