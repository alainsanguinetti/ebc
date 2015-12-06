#!/usr/bin/env python

""" This scripts is the first attempt to create a controller node for our project.

It is in charge of interpreting the simulation parameters, reacting to events from 
the robots and sending back control events. 

May the Force be with us. """

from Component import *

class Controller ( Component ):
    """ This is a controller. It is a runnable component of our project """ 

    def loopHook(self):
    
        print "this is awesome" 
                
                
            
            
if __name__ == "__main__":
    ctrl = Controller ( "controller" )
    ctrl.start()
    ctrl.loop(1)
