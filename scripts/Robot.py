#!/usr/bin/env python

""" This file defines the robot object as seen from the controller point of view. It is a representation of a robot in the simulation """

from Component import Component
from Task import Task
import rospy
from ebc.msg import Event
from ebc.msg import Task as TaskMsg
import sys
import json


class Robot ( Component ):
    """ The Robot class is a representation of a robot """

    def loadConfig( self ):
        """ This method load the config file """

        config_file_n = rospy.get_param( 'config_file_name' )

        try:
            self.config = json.load( open ( config_file_n, 'r' ) )

        except:
            e = sys.exc_info()[0]
            self.display("Error opening config file. %s" % e)
            raise



    def sectorLength( self ):
        """ This method returns length of a sector for a certain id """
        
        for sector in self.config['graph']['sectors']:
            if sector ['id']== self.sector_id:
                return sector['length']
                
                

    def controller_event_callback ( self, message ):
        """ This method is called when the robot receives a message from the controller """

        # Handle the different kind of event
        
        self.display( "ctrlr_evnt_callback: " + str(message.robot_id) + " " + str(message.event) )
        self.display( "robot state: " + str( self.id ) + " " + str( self.length ) )

        
        if self.task:
            if self.id == message.robot_id and self.length == 0 and message.event==Event.G:
                self.length = self.sectorLength()
                self.event_publisher.publish( Event( self.id, Event.B ) )
            return



    def task_cb( self, message ):
        """
        Handles the arrival of task messages
        """
        self.display ( str( message.robot_id ) + " " + message.sectors )
        # Store the message as the current task
        if message.robot_id == self.id:
            self.task = Task( message.sectors )

            self.display( "Received a task" )
            
            self.sector_id = self.task.sectors[ 0 ]
            self.length = self.sectorLength()



    def setup ( self ):
        """ Creates the subscribers and publishers and all needed variables (with self.name_of_the_variable) """

        # Get our robot id
        self.loadConfig()
        self.name = rospy.get_name()
        self.id = int(filter(str.isdigit, self.name))

        # Creates the publisher for events of the robot
        self.event_publisher = rospy.Publisher( rospy.get_name(), Event )

        # Creates the subscriber for events from the controller and link them to the callback
        self.controller_subscriber = rospy.Subscriber ( rospy.get_param( 'controller_name' ), Event, self.controller_event_callback )

        # Subscriber for task messages
        self.task = []
        self.task_sub = rospy.Subscriber( "/tasks", TaskMsg, self.task_cb, queue_size=10 )
        
        # Publisher for the state of the robot
        
        self.length = 0
        
        return
        

    def loopHook ( self ):
        """ This method will be called repetetively while the robot is on """
        s=0
        id =  self.id

        # simulate the robot moving if it is in moving state IF it has permission
        if self.length == 0:
            self.event_publisher.publish( Event( self.id, Event.EOS ) )

        elif self.length > 0 :
            self.length -= 1

        # wait a bit if it is in waiting state
        # publish the state of the robot
        
        
        rospy.sleep(1)

        return



if __name__ == "__main__":
    robot = Robot ( "robot" )
    robot.start()
    robot.loop(0.1)
