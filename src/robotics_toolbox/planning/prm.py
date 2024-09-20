#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-11-04
#     Author: David Kovar <kovarda8@fel.cvut.cz>
#

"""Module for path planning using Probabilistic Roadmap Method (PRM)."""

from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike
from copy import deepcopy

from scipy.sparse import csr_matrix
from scipy.sparse import csgraph

from robotics_toolbox.core import SE3
from robotics_toolbox.robots.robot_base import RobotBase
from robotics_toolbox.utils import distance_between_configurations, interpolate


class GraphPlanner:
    def __init__(self, graph_matrix: list):
        """module for graph searching"""
        self.scipy_graph = csr_matrix(graph_matrix)
        self.dist_matrix, self.pred = csgraph.shortest_path(
            self.scipy_graph, directed=False, method="FW", return_predecessors=True
        )

    def get_path(self, i, j) -> list:
        """will get path from node i to j as list of nodes IDs visited"""
        path = [j]
        k = j
        while self.pred[i, k] != -9999:
            path.append(self.pred[i, k])
            k = self.pred[i, k]
        return path[::-1]


class Node:
    def __init__(self, id: int, config: ArrayLike | SE2 | SE3):
        self.id = id
        self.config = config
        self.neighbours = []
        self.distance_from_neighbours = []

    def add_neighbour(
        self, neighbour: Node, path_to_neighbour: list[ArrayLike | SE2 | SE3]
    ):
        self.neighbours.append(neighbour)

        self.distance_from_neighbours += [0] * (
            neighbour.id - len(self.distance_from_neighbours)
        ) + [len(path_to_neighbour)]


class PRM:
    def __init__(self, robot: RobotBase, delta_q=0.2) -> None:
        """PRM planner for a given robot.
        Args:
            robot: robot used to sample configuration and check collisions
            delta_q: maximum distance between two configurations when building path
        """
        self.robot = robot
        self.delta_q = delta_q
        self.graph = []

    def explore(self, max_nodes: int = 500) -> None:
        """PRM algorithm for motion planning."""

        node_count = 0
        while node_count < max_nodes:
            # pick random points from config space
            q_rand = self.robot.sample_configuration()
            if not self.robot.set_configuration(q_rand).in_collision():
                self.graph.append(Node(id=node_count, config=q_rand))
                node_count += 1
                if node_count == 1:
                    continue
        # connect picked nodes and make graph
        for i, node in enumerate(self.graph):
            for j in range(i, len(self.graph)):
                path = self.connect(q_rand, node.config)
                if path is not None:
                    self.graph[-1].add_neighbour(node, deepcopy(path))
                    path.reverse()
                    node.add_neighbour(self.graph[-1], deepcopy(path))

        # if node wasn't connected remove it from graph
        if len(self.graph[-1].neighbours) == 0:
            self.graph.pop()
            node_count -= 1

    def connect(
        self,
        q_init: ArrayLike | SE2 | SE3,
        q_goal: ArrayLike | SE2 | SE3,
        max_iter: int = 1000,
    ) -> list[ArrayLike | SE2 | SE3] | None:
        """will find path between two given configurations"""

        path = []
        q_new = q_init
        for _ in range(max_iter):
            q_new = interpolate(q_new, q_goal, self.delta_q)

            # if there is collision on path return none
            if self.robot.set_configuration(q_new).in_collision():
                return None

            path.append(q_new)

            # if you get close enough to goal return path
            if distance_between_configurations(q_new, q_goal) < self.delta_q:
                return path

        # if goal is too far away  return none (runs out of iterations)
        return None

    def closest_connect(self, q: ArrayLike | SE2 | SE3, q_to_graph: bool = True):
        """
        connect the closest node to the given configuration

        return: path from the given config to the closest reachable node and closest
        reacheble node id
        """

        closest_node = None
        shortest_path = []
        best_path_len = -1
        for node in self.graph:
            if q_to_graph:
                path = self.connect(q, node.config)
            else:
                path = self.connect(node.config, q)
            if path is None:
                continue
            elif len(path) < best_path_len or best_path_len == -1:
                closest_node = node.id
                shortest_path = deepcopy(path)
                best_path_len = len(shortest_path)

        return shortest_path, closest_node

    def plan(
        self,
        q_start: ArrayLike | SE2 | SE3,
        q_goal: ArrayLike | SE2 | SE3,
        graph_planner: GraphPlanner = GraphPlanner,
    ) -> list[ArrayLike | SE2 | SE3]:
        """will plan path in the current graph"""

        assert len(self.graph) != 0

        # connect init and goal to the graph
        connect_init_path, init_closest_node = self.closest_connect(
            q_start, q_to_graph=True
        )
        assert init_closest_node is not None

        connect_goal_path, goal_closest_node = self.closest_connect(
            q_goal, q_to_graph=False
        )
        assert goal_closest_node is not None

        # construct graph matrix for graph planner
        graph_matrix = []
        for node in self.graph:
            graph_matrix.append(
                node.distance_from_neighbours
                + [0] * (len(self.graph) - len(node.distance_from_neighbours))
            )

        # use graph planner to find a path in the graph
        node_path = graph_planner(graph_matrix).get_path(
            init_closest_node, goal_closest_node
        )

        # create config path from the graph node path
        path = []
        path += connect_init_path
        for i in range(len(node_path) - 1):
            path += [self.graph[node_path[i]].config]
            path += self.connect(
                self.graph[node_path[i]].config, self.graph[node_path[i + 1]].config
            )
        path += connect_goal_path

        return path


if __name__ == "__main__":
    from shapely import MultiPolygon, Point
    from robotics_toolbox.core import SE2
    from robotics_toolbox.robots import PlanarManipulator
    from robotics_toolbox.render import RendererPlanar

    robot = PlanarManipulator(
        link_parameters=[0.3] * 5,
        base_pose=SE2([-0.75, 0.0]),
        structure="RRRRR",
    )
    robot.obstacles = MultiPolygon([Point((0.5, 0.5)).buffer(0.3, cap_style=3)])

    start_state = -np.pi / 4 * np.ones(robot.dof)
    goal_state = np.pi / 4 * np.ones(robot.dof)

    prm = PRM(robot=robot, delta_q=0.2)
    prm.explore(max_nodes=100)
    path = prm.plan(start_state, goal_state)

    render = RendererPlanar(lim_scale=2.0)
    render.plot_manipulator(robot)

    for p in path:
        robot.q = p
        render.redraw_all()

    start_state = -np.pi / 4 * np.ones(robot.dof)
    goal_state = 0 * np.ones(robot.dof)

    path = prm.plan(start_state, goal_state)

    render = RendererPlanar(lim_scale=2.0)
    render.plot_manipulator(robot)

    for p in path:
        robot.q = p
        render.redraw_all()
