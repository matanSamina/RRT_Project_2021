##!/usr/bin/env python

# -*- coding: utf-8 -*-

"""
Created on Thu Sep  5 10:09:21 2019
@author: Matan Samina
"""

from pickle import FALSE, TRUE
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d , ConvexHull, convex_hull_plot_2d



class maps:

    def __init__(self):
        

        rospy.init_node('listener', anonymous=True)
        
        self.started = False  # 'started': if map recived -> true
        self.pub = rospy.Publisher('LM1', numpy_msg(Floats),queue_size=1)  # publisher of landmarks of map
        self.mapLM = None
        self.mapOG = rospy.Subscriber("/map", OccupancyGrid , self.callbackM )

        
        r = rospy.Rate(1) # 0.1 hz
     
        while not rospy.is_shutdown():

            if (self.started):

                self.pub.publish(self.mapLM.ravel())
                points = self.mapLM
                #hull = ConvexHull(points)
                #plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)
                #plt.plot(points[hull.vertices[0],0], points[hull.vertices[0],1], 'ro')
                vor = Voronoi(points, furthest_site = False, incremental=True, qhull_options=None)
                fig = voronoi_plot_2d(vor)
                plt.pause(0.05)

            r.sleep()    
            plt.show()    

    def callbackM(self ,msg):
  
        maps = np.array(msg.data , dtype = np.float32)
        N = np.sqrt(maps.shape)[0].astype(np.int32)
        Re = np.copy(maps.reshape((N,N)))
        
        print(Re)
        #convert to landmarks array
        scale = msg.info.resolution
        #print(Re)
        landMarksArray = (np.argwhere( Re == 100  ) * scale)  # - np.array([CenterShift ,CenterShift]) 
        #plt.scatter(x,y) 
        #len = np.shape(x)[0]
        #points = np.array()
        #for i in x: 
        rospy.loginfo("Landmarks Array of shape " + str(landMarksArray.shape[0]))
        self.mapLM = landMarksArray.astype(np.float32) #-np.array()
        if not self.started : self.started = True   


if __name__ == '__main__':

    rospy.loginfo("Occupancy grid to Landmarks Array initiated")
    LM_maps = maps() # convert maps to landmarks arrays
    rospy.spin()