#!/usr/bin/env python

""" This file defines the robot object as seen from the controller point of view. It is a representation of a robot in the simulation """

from Component import Component
from Task import Task
import rospy
from ebc.msg import Event
from ebc.msg import Task as TaskMsg


class Robot ( Component ):

    """ The Robot class is a representation of a robot """
    
    def controller_event_callback ( self, message ):
        """ This method is called when the robot receives a message from the controller """
        
        # Handle the different kind of event
        
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
        
            
    
    def setup ( self ):
        """ Creates the subscribers and publishers and all needed variables (with self.name_of_the_variable) """
        
        # Get our robot id
        self.name = rospy.get_name()
        self.id = int(filter(str.isdigit, self.name))
    
        # Creates the publisher for events of the robot
        self.event_publisher = rospy.Publisher( rospy.get_name(), Event )
        
        # Creates the subscriber for events from the controller and link them to the callback
        self.controller_subscriber = rospy.Subscriber ( rospy.get_param( 'controller_name' ), Event, self.controller_event_callback )
        
        # Subscriber for task messages
        self.task = ""
        self.task_sub = rospy.Subscriber( "/tasks", TaskMsg, self.task_cb, queue_size=10 )
        
        
        return
        
        
        
    def loopHook ( self ):
        """ This method will be called repetetively while the robot is on """
        
        # simulate the robot moving if it is in moving state 
        
        # wait a bit if it is in waiting state
        rospy.sleep(1)
        
        # FOR TESTING 
        #self.display( "Robot is looooooping" )
        self.event_publisher.publish( Event( self.id, Event.B ) )
        
        return
        
        
        
if __name__ == "__main__":
    robot = Robot ( "robot" )
    robot.start()
    robot.loop(1)
    
