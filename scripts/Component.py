#!/usr/bin/env python

""" This files defines the object Component that is a virtual object that defines a runnable component in the project, here a node in ROS """

import rospy 
import sys
import os


class Component ( object ):
    """ A Component object is an object that can be used in a ROS package. It can be initialized and provides a method for shutting down. """

    def __init__ (self, name):
        """ This creates a Component with name "name" """    
        self.name = name 
        
        # Change the working directory to the directory of the script
        os.chdir(os.path.dirname( sys.argv[0]))
        
        
        
    def start( self ):
        """ This creates a ROS node and calls the setup method""" 
           
        rospy.init_node( self.name )
        
        self.setup()
        
        
        
    def setup( self ):
        """ Initialize all variables etc """ 
        return        
        
        
        
    def loop ( self, user_rate=None ):
        """ 
        This is the method where our component iterate endlessly.
       
        Arguments :
        - user_rate -- the frequency at which the loop should iterate, if none is provided the loop will iterate as fast as possible
         """  
        try:
            if not(  user_rate is None ):
                rate = rospy.Rate( user_rate )
            
            while not rospy.is_shutdown():
            
                self.loopHook()
                
                if not(  user_rate is None ):
                    rate.sleep()
                    
        except:
            e = sys.exc_info()[0]
            print ("Error during runtime. %s" % e)  
            raise
            
        finally:
        
            self.stop()           
                    
                    
                    
    def loopHook( self ):
        """ An empty function because it is supposed to be customized by the inheriting object """
        return
        
        
        
    def stop( self ):
        """ If there is something to do before shutting down """ 
        #self.display( "Node is shutting down" )
        return
    
    
    
    def log( self, string ):
        """ Print something to the log file """    
        rospy.logdebug( str( string ) )
        
        
        
    def display( self, string ):
        """ Print something on stdout """    
        rospy.loginfo( rospy.get_name() + ": " + str( string ) )
    
    
    
            
            
