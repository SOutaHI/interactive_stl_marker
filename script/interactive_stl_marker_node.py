#!/usr/bin/env python
# -*- coding: utf-8 -*-

from os import set_blocking
from interactive_stl_marker.generate_interactive_marker import GenerateInteractiveMarkers
import rospy
from interactive_markers.interactive_marker_server import *

def main():
    rospy.init_node("interactive_stl_markers")
    
    server = InteractiveMarkerServer("stl_markers")
    gim = GenerateInteractiveMarkers(server)
    
    rospy.loginfo("initializing..")
    gim.makeCube()
    server.applyChanges()

    rospy.spin()

if __name__=="__main__":
    main()