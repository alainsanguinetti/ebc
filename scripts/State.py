#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This files defines the object State that is used to represent the state of the controller. """

from ebc.msg import Event

class State( object ):
    """ The State object is used to represent a certain setting of the simulation as understood by the controller """
    
    def __init__ ( self, n, m, P=[], R=[], N=[] ):
        """ This function creates a state with P, R and N as attributes """
        self.n = n
        self.m = m  # not sure this is needed
        
        # Creation of the initial state( using n and m)
        if  not P and  not R and not N :
        
            self.P = [0] * n
            self.R = [ "home" ] * n
            self.N = [0] * n 
    
        # Creation of a particular state
        else:
            self.P = P
            self.R = R
            self.N = N
            
            
            
    def checkFreeSectors( self ):
        """
        This will check for which robot the next sector is free
        """
        # For all robots, check if the next sector is free
        for i in range( 0, self.n ):
            all_clear = True
            next_sector = self.N[ i ]
            for j in range( 0, self.n ):
                if self.R[j] == next_sector:
                    self.active_robots[ i ] = 0 # This robot cannot be active
                    
            
            
    def checkHighestCompletion( self ):
        """
        This apply the rule that the robot with the highest advancement 
        has priority
        """
        adv_max = 0
        # Find the highest advancement
        for i in range( 0, self.n ):
            if self.active_robots[ i ] == 1:
                if self.P[ i ] > adv_max:
                    adv_max = self.P[ i ]
                    
        # Only robots with this advancement are valid
        for i in range( 0, self.n ):
            if self.P[ i ] < adv_max:
                self.active_robots[ i ] = 0 # Robot not active
                
                
                
    def firstActiveRobot( self ):
        """
        Return the index of the first active robot
        """
        for i in range( 0, self.n ):
            if self.active_robots[ i ] == 1:
                return i    # the index of the first active robot
                
        return None # ===> Deadlock
                
            
            
    def nextEvent( self ):
        """
        Returns the first active event for the given state
        """
        self.active_robots = [1]*self.n # list of robot that can be activated
        # Apply rule to see if next sector is free
        self.checkFreeSectors()
        
        # Apply rule to check for the highest completion
        self.checkHighestCompletion()
        
        # Apply rule to check the lowest index of robot
        robot_index = self.firstActiveRobot()
        
        if robot_index != None:
            return Event( robot_index, Event.G )
            
        return
    
            
    def __str__( self ):
        """
        Prints the state
        """
        return "State: \n" + str( self.P ) + "\n" + str( self.R ) + "\n" +  str( self.N )
