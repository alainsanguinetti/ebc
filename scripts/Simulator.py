#!/usr/bin/env python

""" This files defines the object Component that is a virtual object that defines a runnable component in the project, here a node in ROS """

import rospy
import sys
import os
from Tkinter import *
import ttk

from Component import Component


class Simulator ( Component ):
    """ A Component object is an object that can be used in a ROS package. It can be initialized and provides a method for shutting down. """




    def setup( self ):
        """ Initialize all variables etc """

        # Subscriber for robots states
        #self.robot_subscriber = rospy.Subscriber ( rospy.get_param( 'robot_name' ), Event )

        # Subscribers

        # Set up display

        root = Tk()
        root.title("Simulator")

        mainframe = ttk.Frame(root, padding="5 5 12 12")
        mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
        mainframe.columnconfigure(0, weight=1)
        mainframe.rowconfigure(0, weight=1)

        #StringVar()
        cSector1 = StringVar()
        cSector1.set("S1")
        cSector2 = StringVar()
        cSector2.set("S4")
        cSector3 = StringVar()
        cSector3.set("S2")


        nSector1 = StringVar()
        nSector1.set("S2")
        nSector2 = StringVar()
        nSector2.set("S5")
        nSector3 = StringVar()
        nSector3.set("S3")

        event1 = StringVar()
        event1.set("G")
        event2 = StringVar()
        event2.set("B")
        event3 = StringVar()
        event3.set("EOS")

        #IntVar()
        length1 = IntVar()
        length1.set(2)
        length2 = IntVar()
        length2.set(1)
        length3 = IntVar()
        length3.set(3)

        #Labels
        ttk.Label(mainframe, text=" Robot # ",relief=GROOVE,width=15, anchor = CENTER).grid(column=1, row=1, sticky=W)
        ttk.Label(mainframe, text=" Current sector ",relief=GROOVE,width=15, anchor = CENTER).grid(column=2, row=1, sticky=W)
        ttk.Label(mainframe, text=" Next sector ",relief=GROOVE,width=15, anchor = CENTER).grid(column=3, row=1, sticky=W)
        ttk.Label(mainframe, text=" Remaining length ",relief=GROOVE,width=15, anchor = CENTER).grid(column=4, row=1, sticky=W)
        ttk.Label(mainframe, text=" Event ",relief=GROOVE,width=15, anchor = CENTER).grid(column=5, row=1, sticky=W)

        #Robots in system
        ttk.Label(mainframe, text="Robot 1", anchor = CENTER).grid(column=1, row=2, sticky=W)
        ttk.Label(mainframe, text="Robot 2", anchor = CENTER).grid(column=1, row=3, sticky=W)
        ttk.Label(mainframe, text="Robot 3", anchor = CENTER).grid(column=1, row=4, sticky=W)

        # Current status
        ttk.Label(mainframe, textvariable=cSector1, anchor = CENTER).grid(column=2, row=2, sticky=(W, E))
        ttk.Label(mainframe, textvariable=cSector2, anchor = CENTER).grid(column=2, row=3, sticky=(W, E))
        ttk.Label(mainframe, textvariable=cSector3, anchor = CENTER).grid(column=2, row=4, sticky=(W, E))

        # Next status
        ttk.Label(mainframe, textvariable=nSector1, anchor = CENTER).grid(column=3, row=2, sticky=(W, E))
        ttk.Label(mainframe, textvariable=nSector2, anchor = CENTER).grid(column=3, row=3, sticky=(W, E))
        ttk.Label(mainframe, textvariable=nSector3, anchor = CENTER).grid(column=3, row=4, sticky=(W, E))

        # remaining length
        ttk.Label(mainframe, textvariable=length1, anchor = CENTER).grid(column=4, row=2, sticky=(W, E))
        ttk.Label(mainframe, textvariable=length2, anchor = CENTER).grid(column=4, row=3, sticky=(W, E))
        ttk.Label(mainframe, textvariable=length3, anchor = CENTER).grid(column=4, row=4, sticky=(W, E))

        # current event
        ttk.Label(mainframe, textvariable=event1, anchor = CENTER).grid(column=5, row=2, sticky=(W, E))
        ttk.Label(mainframe, textvariable=event2, anchor = CENTER).grid(column=5, row=3, sticky=(W, E))
        ttk.Label(mainframe, textvariable=event3, anchor = CENTER).grid(column=5, row=4, sticky=(W, E))

        for child in mainframe.winfo_children(): child.grid_configure(padx=5, pady=5)

        root.bind('<Return>', '''calculate''')
        root.mainloop()


        return


    def loopHook( self ):

        # refresh display
        """ An empty function because it is supposed to be customized by the inheriting object """



        return

    def stop( self ):
        """ If there is something to do before shutting down """
        #self.display( "Node is shutting down" )
        return


if __name__ == "__main__":
    sim = Simulator ( "simulator" )
    sim.start()
    sim.loop(1)
