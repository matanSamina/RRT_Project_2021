#!/usr/bin/env python

# -*- coding: utf-8 -*-

"""
Created on Thu Sep  5 10:09:21 2019
@author: Matan Samina
"""

import numpy as np


def heuristic(from_node, to_node):
    return np.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)


class Node:
    def __init__(self, ID, x, y, neighbours):
        self.ID = ID
        self.x = x
        self.y = y
        self.neighbours = neighbours
        self.parent = None
        self.g = 0
        self.h = 0
        self.w = 0 
        self.cov = None

    def f(self):
        return self.g + self.h


def get_lowest_f_node_from(open_list):
    lowest_node = open_list[0]
    for node in open_list:
        if node.f() < lowest_node.f():
            lowest_node = node
    return lowest_node


def get_node(successor_ID, nodes):
    for node in nodes:
        if node.ID == successor_ID:
            return node
    return None


def way(node_current, node_successor):
    return heuristic(node_current, node_successor)


def a_star(start, goal, nodes):
    open_list = []
    close_list = []
    node_current = start
    node_current.h = heuristic(start, goal)
    open_list.append(start)
    while len(open_list) > 0:
        node_current = get_lowest_f_node_from(open_list)
        if node_current.ID == goal.ID:
            break
        for successor_ID in node_current.neighbours:
            node_successor = get_node(successor_ID, nodes)
            successor_current_cost = node_current.g + way(node_current, node_successor)
            if node_successor in open_list:
                if node_successor.g <= successor_current_cost:
                    continue
            elif node_successor in close_list:
                if node_successor.g <= successor_current_cost:
                    continue
                close_list.remove(node_successor)
                open_list.append(node_successor)
            else:
                open_list.append(node_successor)
                node_successor.h = heuristic(node_successor, goal)
            node_successor.g = successor_current_cost
            node_successor.parent = node_current

        open_list.remove(node_current)
        close_list.append(node_current)

    if node_current.ID != goal.ID:
        return None
    else:
        path = []
        while node_current is not None:
            path.append(node_current)
            node_current = node_current.parent
        return path



