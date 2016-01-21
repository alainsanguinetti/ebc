#!/usr/bin/env python

""" This files defines the object Component that is a virtual object that defines a runnable component in the project, here a node in ROS """

import rospy 
import sys
import os

from Component import Component


class Simulator ( Component ):
    """ A Component object is an object that can be used in a ROS package. It can be initialized and provides a method for shutting down. """
        
        
        
    def setup( self ):
        """ Initialize all variables etc """ 
        
        # Subscriber for robots states
        
        # Subscribers 
        
        # Set up display
        
        
        return        
        
        
    def loopHook( self ):
    
        # refresh display
        """ An empty function because it is supposed to be customized by the inheriting object """
        return
        
        
        
    def stop( self ):
        """ If there is something to do before shutting down """ 
        #self.display( "Node is shutting down" )
        return
        
            
if __name__ == "__main__":
    sim = Simulator ( "simulator" )
    sim.start()
    sim.loop(1)            
