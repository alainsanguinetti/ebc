#!/usr/bin/env python

""" This scripts is the first attempt to create a controller node for our project.

It is in charge of interpreting the simulation parameters, reacting to events from 
the robots and sending back control events. 

May the Force be with us. """

from Component import Component
from Graph import Graph
import rospy
import sys
import json
import roslaunch

class Controller ( Component ):
    """ This is a controller. It is a runnable component of our project """ 
    
    
    
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
        
        
        
    def createRobots( self ):
        """
        This method will creates all the node corresponding to robots
        and store process handlers in an array
        """
        
        package = "ebc"
        executable = "Robot.py"
        name_base = "robot"
        
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        
        self.robot_processes = []
        for i in range( 0, self.config["performers"]["quantity"] ):
            # Load the parameter to be passed to the robot 
            #launch.load_str( "<node name=\"robot" + str(i) + "\" pkg=\"ebc\" type=\"Robot.py\" output=\"screen\" ></node>")
            robot_process = launch.launch( roslaunch.core.Node(package, executable, name=name_base+str(i) ) )
            
            self.robot_processes.append( robot_process )
            
            
        
    
    def setup ( self ):
        """ Here we set up the simulation parameters for the controller """
        
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
        # Stop robot processes
        for robot_process in self.robot_processes:
            robot_process.stop()
            
            
            
            
if __name__ == "__main__":
    ctrl = Controller ( "controller" )
    ctrl.start()
    ctrl.loop(1)
