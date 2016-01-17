#!/usr/bin/env python
# -*- coding:utf-8 -*-

""" This scripts is the first attempt to create a controller node for our project.

It is in charge of interpreting the simulation parameters, reacting to events from 
the robots and sending back control events. 

May the Force be with us. """

from Component import Component
from Graph import Graph
from RobotHandler import RobotHandler
from ebc.msg import Event
import rospy
import sys
import json
import roslaunch



class Controller ( Component ):
    """
    This is a controller. It is a runnable component of our project.
    It ensures that the system will perform the tasks.
    """ 
    
    def loadConfig( self ):
        """ This method load the config file """
        
        config_file_n = rospy.get_param( 'config_file_name' )
        
        try:
            self.config = json.load( open ( config_file_n, 'r' ) )
            
        except:
            e = sys.exc_info()[0]
            self.display("Error opening config file. %s" % e)
            raise
            
            
            
    def createListGraph( self, sectors ):
        """
        This method will create the list of Graph object that will represent the environment
        
        Returns :
        - graph  -- a list of Graph objects
        """
        graph = []
        for sector in sectors:
        
            graph.append( Graph( sector ) )
            
        return graph
            
            
            
    def linkGraphs ( self ):
        """ 
        This method will link all the Graph objects
        """
        
        # Link all graphs object with their nexts sectors
        for graph in self.graph:
        
            next = []
            
            for next_id in graph.next_ids:
            
                for next_graph in self.graph:
                
                    if next_graph.id == next_id:
                    
                        next.append( next_graph )
                        
            graph.next = next
            
        # Now we only need the reference of the home sectors to describe the whole graph
        self.graph = self.graph[0] 

        
            
    def loadGraph( self ):
        """ This method creates the graph object to represent the environment """

        sectors = self.config[ 'graph' ]['sectors'] # To this date we only use the sectors
        self.graph = self.createListGraph( sectors )
        self.linkGraphs()
        
        
        
    def robot_callback( self, msg ):
        """
        Reacts to a event from a robot
        """
        
        self.display ( str(msg.robot_id) + ": " + str( msg.event ) )
        
        
        
    def createRobots( self ):
        """
        This method will creates all the node corresponding to robots
        and store process handlers in an array.
        It will also creates the corresponding topics.
        """
        
        package = "ebc"
        executable = "Robot.py"
        name_base = "robot"
        
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        
        self.robot_handlers = []
        for i in range( 0, self.config["performers"]["quantity"] ):
        
            self.robot_handlers.append( RobotHandler( name_base+str(i), launch, self ) )
            
            
        
    
    def setup ( self ):
        """ Here we set up the simulation parameters for the controller """
        
        # Publish our name to the parameters server
        rospy.set_param( 'controller_name', rospy.get_name() )
        
        # Create our control publish
        self.pub = rospy.Publisher( rospy.get_name(), Event )
        
        self.loadConfig() # Load the config file
        
        # creates the graph object
        self.loadGraph()
        
        self.display( self.graph.id )
        self.display( self.graph.next[0].next[0].id )
        
        # creates the robot objects
        self.createRobots()
        
        # initialize the state
        


    def loopHook(self):
        """ The loop hook is called by the loop() method """
    
        self.display(  "this is awesome" )
                
                
                
    def stop( self ):
        """
        This is the cleanup function
        """
            
        for robot_handler in self.robot_handlers:
            robot_handler.stop()
            
            
            
            
if __name__ == "__main__":
    ctrl = Controller ( "controller" )
    ctrl.start()
    ctrl.loop(1)
