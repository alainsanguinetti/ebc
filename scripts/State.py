#!/usr/bin/env python


""" This files defines the object State that is used to represent the state of the controller. """

class State( object ):
    """ The State object is used to represent a certain setting of the simulation as understood by the controller """
    
    def __init__ ( self, P, R, N ):
        """ This function creates a state with P, R and N as attributes """
        
        self.P = P
        self.R = R
        self.N = N
    
