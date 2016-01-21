#!/usr/bin/env python 
# -*- coding: utf-8 -*-

"""
This files defines a task as an object
"""

class Task ( object ):
    """
    A task stores a sequence of sectors and a status
    """
    
    def __init__( self, sequence ):
        """
        Creates the task from a string containing the sequence of sectors
        
        sequence -- a string defining the sequence of sectors
        """
        
        self.sectors = sequence.split( "/" ) # Delimiters of sectors
        self.sequence = sequence
        
        
    def next( self ):
        """
        Update the status of the task to the next sector
        """
        
        return
        
        
    
    def toList( self ):
        """
        Returns the task as a list of sectors to go through
        """
        
        return self.sectors
        
        
        
    def __str__( self ):
        """
        Return a string defining the task
        """
        return self.sequence
