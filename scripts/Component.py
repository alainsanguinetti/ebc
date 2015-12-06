#!/usr/bin/env python

""" This files defines the object Component that is a virtual object that defines a runnable component in the project, here a node in ROS """

import rospy 


class Component ( object ):
    """ A Component object is an object that can be used in a ROS package. It can be initialized and provides a method for shutting down. """

    def __init__ (self, name):
        """ This creates a Component with name "name" """    
        self.name = name
        
        
    def start( self ):
        """ This creates a ROS node """ 
           
        rospy.init_node( self.name )
        
        
    def loop ( self, user_rate=None ):
        """ 
        This is the method where our component iterate endlessly.
        
        user_rate -- the frequency at which the loop should iterate, if none is provided the loop will iterate as fast as possible
         """  
        
        if not(  user_rate is None ):
            rate = rospy.Rate( user_rate )
        
        while not rospy.is_shutdown():
        
            self.loopHook()
            
            if not(  user_rate is None ):
                rate.sleep()
            
            
    def loopHook( self ):
        return
    
            
            
