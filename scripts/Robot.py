#!/usr/bin/env python

""" This file defines the robot object as seen from the controller point of view. It is a representation of a robot in the simulation """

from Component import Component

class Robot ( Component ):

    """ The Robot class is a representation of a robot """
    
    def controller_event_callback ( self, message ):
        """ This method is called when the robot receives a message from the controller """
        
        # Handle the different kind of event
        
        return
        
            
    
    def setup ( self ):
        """ Creates the subscribers and publishers and all needed variables (with self.name_of_the_variable) """
    
        # Creates the publisher for events of the robot
#        self.event_publisher = 
        
        # Creates the subscriber for events from the controller and link them to callbacks
#        self.controller_subscriber = 
        
        return
        
        
        
    def loopHook ( self ):
        """ This method will be called repetetively while the robot is on """
        
        # simulate the robot moving if it is in moving state 
        
        # wait a bit if it is in waiting state
        
        # FOR TESTING 
        self.display( "Robot is looooooping" )
        
        return
        
        
        
if __name__ == "__main__":
    robot = Robot ( "robot" )
    robot.start()
    robot.loop(1)
    
