#!/usr/bin/env python

""" This scripts is the first attempt to create a controller node for our project.

It is in charge of interpreting the simulation parameters, reacting to events from 
the robots and sending back control events. 

May the Force be with us. """

from Component import *
import sys
import os

class Controller ( Component ):
    """ This is a controller. It is a runnable component of our project """ 
    
    def setup ( self ):
        """ Here we set up the simulation parameters for the controller """
        config_file_n = rospy.get_param( 'config_file_name' )
        
        try:
            config_file = open ( config_file_n, 'r' )
            
        except:
            e = sys.exc_info()[0]
            self.display("Error opening config file. %s" % e)
            raise

    def loopHook(self):
        """ The loop hook is called by the loop() method """
    
        self.display(  "this is awesome" )
                
                
            
            
if __name__ == "__main__":
    ctrl = Controller ( "controller" )
    ctrl.start()
    ctrl.loop(1)
