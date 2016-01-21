#!/usr/bin/env python
# -*- coding:utf-8 -*-

""" This scripts is the first attempt to create a controller node for our project.

It is in charge of interpreting the simulation parameters, reacting to events from 
the robots and sending back control events. 

May the Force be with us. """

from Component import Component
from Graph import Graph
from State import State
from Task import Task
from RobotHandler import RobotHandler
from ebc.msg import Event
from ebc.msg import Task as TaskMsg
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
        Reacts to a event from a robot : updates the state
        """
        # If the robot crosses a border, we change the state
        # if the robot is displaying EOS or Gr, we do not change the state of the controller
        if msg.event != Event.EOS and msg.event != Event.G:
            # update the state
            
            # If the robot has finished the task
            if not self.robot_handlers[msg.robot_id].task:
                self.state.P[msg.robot_id] = 0
                self.state.R[msg.robot_id] = self.robot_handlers[msg.robot_id].nextSector() # returns the "home" value
                self.state.N[msg.robot_id] = self.robot_handlers[msg.robot_id].nextSector()            
                
            else:
                self.state.P[msg.robot_id] += 1
                self.state.R[msg.robot_id] = self.state.N[msg.robot_id]
                self.state.N[msg.robot_id] = self.robot_handlers[msg.robot_id].nextSector()
                
                self.robot_handlers[msg.robot_id].changeSector()
               
        self.display ( "Received event from " + str(msg.robot_id) + ": " + str( msg.event ) )
        
        
        
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

        return
            
            
    
    def loadTasks( self ):
        """
        This load the task and creates a stack of tasks
        """
        self.tasks = []
        for task in self.config["tasks"]:
            self.tasks.append( Task( task ) )
            
        self.next_assigned_task_index = 0
        
        return
        
                
        
    def sendTask( self, robot, task ):
        """
        Assign and send a task to the robot
        """
        # Wait until all robots are available
        while not ( self.task_pub.get_num_connections() >= self.config["performers"]["quantity"] ):
            rospy.sleep( 0.1 )
            
        # publish message
        self.task_pub.publish( robot.id, str(task) )
        
        self.robot_handlers[robot.id].assignTask( task, self.next_assigned_task_index )
    
        self.display( "Sent task " 
                    + str(self.next_assigned_task_index)
                    + " to robot " 
                    + str(robot.id) )
            
            
            
    def dispatchTasks( self ):
        """
        This method will take the unfinished tasks from the pool of tasks and 
        assign them to unoccupied robots
        """
        # For each unoccupied robots
        for robot in self.robot_handlers:
            if robot.acceptTask:
                # assign an unassigned task( if there are any)
                if self.next_assigned_task_index < len( self.tasks ): 
                    self.sendTask( robot, self.tasks[ self.next_assigned_task_index ] )
                    self.next_assigned_task_index += 1
                else:
                    break
            
        
    
    def setup ( self ):
        """ Here we set up the simulation parameters for the controller """
        self.loadConfig() # Load the config file
        
        # creates the robot handler objects
        self.createRobots()
        
        # Publish our name to the parameters server
        rospy.set_param( 'controller_name', rospy.get_name() )
        
        # Create our control publish
        self.event_pub = rospy.Publisher( rospy.get_name(), Event )
        
        # creates the graph object
        self.loadGraph()
        
        # Creates and dispatch the task messages
        self.task_pub = rospy.Publisher( "/tasks", TaskMsg, latch=True ) # important to latch
        self.loadTasks()
        self.dispatchTasks()
                
        # initialize the state
        self.state = State( len( self.robot_handlers ), len( self.config["graph"] ) )
        


    def loopHook(self):
        """ The loop hook is called by the loop() method """
        
        # See if we can fire any control event (grant)
        event = self.state.nextEvent()
        if event:
            self.event_pub.publish( event )
    
        # See if there is any unoccupied robot
        self.dispatchTasks()
        
        # Print and that's it
        self.display( str( self.state ) )
        
        return       
                
                
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
