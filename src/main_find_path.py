#!/usr/bin/env python

# -*- coding: utf-8 -*-

"""
Created on Thu Sep  5 10:09:21 2019
@author: Matan Samina & Arseni the man
"""

# VORONOI -> A-STAR -> MPF -> RRT ->  RRT PATH

from numpy.core.fromnumeric import argmax, shape
from numpy.core.function_base import linspace
from tf import transformations
from pickle import FALSE, NONE, TRUE
import math
from numpy.core.defchararray import array
from numpy.core.multiarray import result_type
from numpy.core.numeric import Inf
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rospy.client import _init_node_params
from rospy.numpy_msg import numpy_msg
import matplotlib.pyplot as plt
from tuw_multi_robot_msgs.msg import Graph
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from astar import Node, a_star
from RRT import NodeRRT, rrt_mpf, plotRRT
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.stats import multivariate_normal


def calcProb(nodesAstar, point):
    """
    calculate to propbaility of point in the map
    """
    dist = 1e6
    for node in nodesAstar:

        mean = np.array([node.x, node.y])
        distFromNode = np.linalg.norm(mean - point)

        if distFromNode < dist:
            minDistNode = node
            dist = distFromNode

    y = multivariate_normal.pdf(point, mean=np.array([minDistNode.x, minDistNode.y]),
                                cov=minDistNode.cov)  # *minDistNode.w
    return y


def MPFsampling(nodesAstar, xmin, xmax, ymin, ymax):
    """
    :return: x, y from the distribution
    """
    Nsampels = 20
    xarr = np.random.uniform(xmin, xmax, Nsampels)
    yarr = np.random.uniform(ymin, ymax, Nsampels)

    prob = []
    dict_of_points = {}
    sum = 0

    for i in xarr:
        for j in yarr:
            prob = calcProb(nodesAstar, np.array([i, j]))
            dict_of_points[(i, j)] = prob
            sum += prob

    sumb = 0
    probmax = 0
    x, y = 0, 0
    for k, v in dict_of_points.items():
        sumb += v / sum
        if v > probmax:
            probmax = v
            x, y = k
        color = 'red' if v / sum > 0.001 else 'blue'
        # plt.scatter(k[0] , k[1] , color=color , alpha=0.1 )

    # Resampling
    points = []
    prob = []
    for k, v in dict_of_points.items():
        points.append(k)
        prob.append(v / sum)
    # print(f'sum of prob: {np.sum(prob)}')
    sample = np.random.choice(list(range(len(points))), 1, p=prob)
    # print(f'points {points} , ans ample {sample}')
    x = points[sample[0]][0]
    y = points[sample[0]][1]
    # print(f'arseni x y  is {x , y} ')
    # plt.scatter(x,y ,color= "red")
    # plt.show()
    return x, y


def getX(z):
    return z.x


def getY(z):
    return z.y


def rotate(map, angle):  # rotat map for DE

    theta = np.radians(angle)  # angles to radians
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))  # Rotation matrix
    RotatedLandmarks = np.matmul(map, R)  # + np.array(dx, dy) # matrix multiplation

    return RotatedLandmarks


class voronoi:
    """
    Main ROS node
    """
    def __init__(self):

        rospy.init_node('Graph', anonymous=True)

        self.ver = None
        self.start = False
        self.Target = False
        self.started = False  # 'started': if map recived -> true
        self.mapLM = None

        rospy.Subscriber("/clicked_point", PointStamped, self.callbackTarget)
        rospy.Subscriber("/odom", Odometry, self.callbackOdom)
        rospy.Subscriber("/map", OccupancyGrid, self.callbackMap)
        rospy.Subscriber("/segments", Graph, self.callbackM)
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=3)
        self.publishvel = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

        self.voronoiGraph = False
        r = rospy.Rate(0.1)  # 0.1 hz

        while not rospy.is_shutdown():

            nodes = None

            if self.Target and self.voronoiGraph:

                self.plotVoroni()
                if self.mapLM is not None: self.plotMap()

                # plt.show() #map + voronoi

                # plt.scatter(self.Tx , self.Ty)
                result = self.result

                self.plotVoroni()
                mapR = self.plotMap()
                # plot path created by Astar on voronoi graph  
                if self.result is not None: self.plotAstar(result)

                # plt.show() # map + voronoi + astar

                self.plotMap()
                self.plotAstar(result)

                resultLen = len(result)
                resultRe = result[::-1]
                d = 0.1
                # wigths for sampling algorithm (MPF)
                wigths = np.linspace(0, 1000, resultLen)
                for node, W in zip(resultRe, wigths):
                    node.w = W
                    # add cov matrix 
                    node.cov = np.array([[d / 3, 0], [0, d / 3]])

                # UNIFORM sampling
                xmin, xmax = min(mapR[:, 0]), max(mapR[:, 0])
                ymin, ymax = min(mapR[:, 1]), max(mapR[:, 1])

                nodes, sampled_points, status = rrt_mpf(nodes=[NodeRRT(x=self.Sx, y=self.Sy)],
                                                        number_of_samples=100,
                                                        LAMBDA=1.0,
                                                        step=0.2,
                                                        x_goal=self.Tx, y_goal=self.Ty,
                                                        x_obst_list=mapR[:, 0], y_obst_list=mapR[:, 1],
                                                        mapX_max=xmax, mapX_min=xmin,
                                                        mapY_max=ymax, mapY_min=ymin,
                                                        sample_func=MPFsampling,
                                                        nodesAstar=resultRe)

                print(f'Finish with {status} \n There were {len(sampled_points)} Sampeled points')

                xarr = np.linspace(xmin, xmax, 30)
                yarr = np.linspace(ymin, ymax, 30)

                dict_of_points = {}
                sum = 0
                for i in xarr:
                    for j in yarr:
                        prob = calcProb(resultRe, np.array([i, j]))
                        dict_of_points[(i, j)] = prob
                        sum += prob

                sumb = 0

                for k, v in dict_of_points.items():
                    sumb += v / sum
                    color = 'red' if v / sum > 0.001 else 'blue'
                    plt.scatter(k[0], k[1], color=color, alpha=0.5)
                    # print(f"prob: {v/sum}")

                # plt.show() # map + Astar + MPF

                self.plotMap()
                print(f' sum of probabilities is {sumb}')

                plotRRT(nodes, sampled_points, status)

                pathRRT = []
                if status == 'success':
                    path_node = nodes[-1]

                    while path_node.parentNode:
                        # node.parentXY
                        pathRRT.append(path_node)
                        parent = path_node.parentNode
                        path_node = parent

                # move robot on the path of Astar algorithm
                if status == "success":
                    self.move2(pathRRT)

                self.Target = False

    def plotAstar(self, result):
        parent = result[0]
        successor = parent
        for node in result:
            parent = node
            plt.plot([successor.x, parent.x], [successor.y, parent.y], linewidth=5, color='green')
            successor = node

    def plotVoroni(self):
        # PLOT VORONOI
        for path in self.ver:
            xList = list(map(getX, path.path))
            yList = list(map(getY, path.path))
            plt.scatter(xList, yList, c="gray")

    def plotMap(self):

        maplM = self.mapLM
        mapR = rotate(maplM, 270)
        mapR[:, 0] = -1 * mapR[:, 0]
        plt.scatter(mapR[:, 0], mapR[:, 1])
        return mapR

    def move2(self, nodes):
        """
        This method actually moves the robot
        """
        nodes = nodes[::-1]

        stepCounter = 0

        for nodePath in nodes:
            Tx = nodePath.x
            Ty = nodePath.y

            print(f' x: {nodePath.x} , y : {nodePath.y}')

            cmdVel = Twist()
            ww = math.atan2((Ty - self.Sy), (Tx - self.Sx))
            ww = math.degrees(ww)
            print("move turn")
            dist = 100

            stepCounter += 1

            while (np.abs(dist) > 0.05):
                err = 10
                while (np.abs(err) > 1):
                    # print(self.orientation)
                    orList = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
                    (_, _, w) = transformations.euler_from_quaternion(orList)
                    ww = math.atan2((Ty - self.Sy), (Tx - self.Sx))
                    err = self.err(ww, w)
                    cmdVel.angular.z = np.sign(err) * np.abs(err) / 18
                    self.publishvel.publish(cmdVel)

                cmdVel.angular.z = 0
                dist = math.dist([self.Sx, self.Sy], [Tx, Ty])
                # print(f'dist:{dist}')
                cmdVel.linear.x = min(0.3, np.abs(dist) / 2.5)
                # print(min(0.3, np.abs(dist)/3))
                self.publishvel.publish(cmdVel)

                # print(f'dist : {dist}')

        rospy.loginfo("move finishd")
        rospy.loginfo(f'num of steps is {stepCounter}')

        cmdVel.linear.x = 0
        self.publishvel.publish(cmdVel)

    def err(self, ww, w):
        ww = math.degrees(ww)
        w = math.degrees(w)
        err = ww - w
        err = (err + 180) % 360 - 180
        return err

    def get_rotation(self, msg):

        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    def callbackM(self, msg):

        self.ver = msg.vertices
        self.X_list = []
        self.Y_list = []

        self.nodes = []

        self.started = True
        self.plotTreeVer()

    def plotTreeVer(self):

        nodes = []
        ID = 0

        for path in self.ver:

            IDlist = range(ID, ID + len(path.path))
            xList = list(map(getX, path.path))
            yList = list(map(getY, path.path))
            # plt.scatter(xList, yList)

            for x, y in zip(xList, yList):
                nei = []
                if ID + 1 in IDlist: nei.append(ID + 1)
                if ID - 1 in IDlist: nei.append(ID - 1)
                nodes.append(Node(ID, x, y, nei))
                ID += 1

        for node1 in nodes:
            for node2 in nodes:
                if [node1.x, node1.y] == [node2.x, node2.y]:
                    if node1.ID != node2.ID:
                        node1.neighbours.append(node2.ID)
                        node2.neighbours.append(node1.ID)

        print(f'number of nodes : {len(nodes)}')
        self.nodes = nodes
        self.voronoiGraph = True

    def callbackTarget(self, msg):

        # self.movebase_client( msg.point.x, msg.point.y)

        pose = PoseStamped()
        pose.pose.position.x = 9
        pose.pose.position.y = 9
        pose.pose.orientation.w = 1.0

        print("im here")

        self.Tx = msg.point.x + 10
        self.Ty = msg.point.y + 10

        if self.voronoiGraph:

            print(f'xtarget : {self.Tx} {self.Ty}')
            self.Target = True

            minDist = 10000
            IDofMin = -1

            IDofMinR = -1
            minDistR = 10000

            for node in self.nodes:

                dist = math.dist([self.Tx, self.Ty], [node.x, node.y])
                distFromR = math.dist([self.Sx, self.Sy], [node.x, node.y])
                # print(dist)
                if distFromR < minDistR:
                    minDistR, IDofMinR = distFromR, node.ID
                if dist < minDist:
                    minDist, IDofMin = dist, node.ID

            print(f'IDof min : {IDofMin}')
            targetNode = Node(len(self.nodes), self.Tx, self.Ty, [IDofMin])
            self.nodes.append(targetNode)
            self.nodes[IDofMin].neighbours.append(targetNode.ID)
            print(f' ID of target Node : {targetNode.ID} N: {targetNode.neighbours}')
            print(f'fatherNOde {self.nodes[IDofMin].neighbours}')

            node_start, node_goal = self.nodes[IDofMinR], self.nodes[-1]
            result = a_star(node_start, node_goal, self.nodes)
            # print(result)
            self.result = result
            # plot found path

            self.Target = True

    def callbackOdom(self, msg):

        self.orientation = msg.pose.pose.orientation
        self.w = msg.pose.pose.orientation.w
        # self.get_rotation(msg)
        self.Sx = msg.pose.pose.position.x + 10
        self.Sy = msg.pose.pose.position.y + 10
        self.start = True

    def callbackMap(self, msg):

        maps = np.array(msg.data, dtype=np.float32)
        N = np.sqrt(maps.shape)[0].astype(np.int32)
        Re = np.copy(maps.reshape((N, N)))

        # convert to landmarks array
        resolution = msg.info.resolution
        landMarksArray = (np.argwhere(Re == 100) * resolution)  # - np.array([CenterShift ,CenterShift])

        self.mapLM = landMarksArray.astype(np.float32)  # -np.array()


if __name__ == '__main__':
    rospy.loginfo("Occupancy grid to Landmarks Array initiated")
    Vor = voronoi()  # convert maps to landmarks arrays
    rospy.spin()
