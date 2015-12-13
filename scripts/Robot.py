#!/usr/bin/env python

""" This file defines the robot object as seen from the controller point of view. It is a representation of a robot in the simulation """

class Robot ( object ):

    """ The Robot class is a representation of a robot """
    
    def __init__ ( self, topic ):
        """ Creates a Robot object with its corresponding ROS topic """
        self.topic = topic
