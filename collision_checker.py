#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Author: Ryan De Iaco
# Additional Comments: Carlos Wang
# Date: October 29, 2018

import numpy as np
import scipy.spatial
from math import sin, cos, pi, sqrt

class CollisionChecker:
    def __init__(self, circle_offsets, circle_radii, weight):
        self._circle_offsets = circle_offsets
        self._circle_radii   = circle_radii
        self._weight         = weight

    ######################################################
    ######################################################
    # MODULE 7: CHECKING FOR COLLISSIONS
    #   Read over the function comments to familiarize yourself with the
    #   arguments and necessary variables to return. Then follow the TODOs
    #   (top-down) and use the surrounding comments as a guide.
    ######################################################
    ######################################################
    # Takes in a set of paths and obstacles, and returns an array
    # of bools that says whether or not each path is collision free.
    def collision_check(self, paths, obstacles):
        """Returns a bool array on whether each path is collision free.

        args:
            paths: A list of paths in the global frame.  
                A path is a list of points of the following format:
                    [x_points, y_points, t_points]:
                        x_points: List of x values (m)
                        y_points: List of y values (m)
                        t_points: List of yaw values (rad)
                    Example of accessing the ith path, jth point's t value:
                        paths[i][2][j]
            obstacles: A list of [x, y] points that represent points along the
                border of obstacles, in the global frame.
                Format: [[x0, y0],
                         [x1, y1],
                         ...,
                         [xn, yn]]
                , where n is the number of obstacle points and units are [m, m]

        returns:
            collision_check_array: A list of boolean values which classifies
                whether the path is collision-free (true), or not (false). The
                ith index in the collision_check_array list corresponds to the
                ith path in the paths list.
        """
        collision_check_array = np.zeros(len(paths), dtype=bool)
        for i in range(len(paths)):
            collision_free = True
            path           = paths[i]

            # Iterate over the points in the path.
            for j in range(len(path[0])):
                # Compute the circle locations along this point in the path.
                # These circle represent an approximate collision
                # border for the vehicle, which will be used to check
                # for any potential collisions along each path with obstacles.

                # The circle offsets are given by self._circle_offsets.
                # The circle offsets need to placed at each point along the path,
                # with the offset rotated by the yaw of the vehicle.
                # Each path is of the form [[x_values], [y_values],
                # [theta_values]], where each of x_values, y_values, and
                # theta_values are in sequential order.

                # Thus, we need to compute:
                # circle_x = point_x + circle_offset*cos(yaw)
                # circle_y = point_y circle_offset*sin(yaw)
                # for each point along the path.
                # point_x is given by path[0][j], and point _y is given by
                # path[1][j]. 
                # self._circle_offsets例子给的是[-1,1,3],即3个从定位位置沿着纵轴移动的圆心位置的距离
                circle_locations = np.zeros((len(self._circle_offsets), 2))

                # TODO: INSERT YOUR CODE BETWEEN THE DASHED LINES
                # --------------------------------------------------------------
                for p in range(len(self._circle_offsets)):
                    circle_locations[p, 0] = path[0][j] +  self._circle_offsets[p]*cos(path[2][j])
                    circle_locations[p, 1] = path[1][j] +  self._circle_offsets[p]*sin(path[2][j])
                    # print("circle_locations[p, 0] " + str(circle_locations[p, 0])) # 显示圆心坐标
                    # print("circle_locations[p, 1] " + str(circle_locations[p, 1]))
                # --------------------------------------------------------------

                # Assumes each obstacle is approximated by a collection of
                # points of the form [x, y].
                # Here, we will iterate through the obstacle points, and check
                # if any of the obstacle points lies within any of our circles.
                # If so, then the path will collide with an obstacle and
                # the collision_free flag should be set to false for this flag
                for k in range(len(obstacles)):
                    collision_dists = \
                        scipy.spatial.distance.cdist(obstacles[k], 
                                                     circle_locations)
                    collision_dists = np.subtract(collision_dists, 
                                                  self._circle_radii)
                    # print("min collision_dists for obs "+str(k) +" is " + str(np.min(collision_dists))) # 显示碰撞点信息
                    collision_free = collision_free and \
                                     not np.any(collision_dists < 0)
                    # print(' === Path '+str(i) +' Point '+str(j)+' is '+str(collision_free)+' for obs '+str(k) +' === ===')

                    if not collision_free:
                        #print(' === === '+str(i)+' path is collision === ===')
                        break
                if not collision_free:
                    #print(' === ==='+str(i)+' path is collision === ===')
                    break

            collision_check_array[i] = collision_free

        return collision_check_array

    ######################################################
    ######################################################
    # MODULE 7: SELECTING THE BEST PATH INDEX
    #   Read over the function comments to familiarize yourself with the
    #   arguments and necessary variables to return. Then follow the TODOs
    #   (top-down) and use the surrounding comments as a guide.
    ######################################################
    ######################################################
    # Selects the best path in the path set, according to how closely
    # it follows the lane centerline, and how far away it is from other
    # paths that are in collision. 
    # Disqualifies paths that collide with obstacles from the selection
    # process.
    # collision_check_array contains True at index i if paths[i] is
    # collision-free, otherwise it contains False.
    def select_best_path_index(self, paths, collision_check_array, goal_state):
        """Returns the path index which is best suited for the vehicle to
        traverse.

        Selects a path index which is closest to the center line as well as far
        away from collision paths.

        args:
            paths: A list of paths in the global frame.  
                A path is a list of points of the following format:
                    [x_points, y_points, t_points]:
                        x_points: List of x values (m)
                        y_points: List of y values (m)
                        t_points: List of yaw values (rad)
                    Example of accessing the ith path, jth point's t value:
                        paths[i][2][j]
            collision_check_array: A list of boolean values which classifies
                whether the path is collision-free (true), or not (false). The
                ith index in the collision_check_array list corresponds to the
                ith path in the paths list.
            goal_state: Goal state for the vehicle to reach (centerline goal).
                format: [x_goal, y_goal, v_goal], unit: [m, m, m/s]
        useful variables:
            self._weight: Weight that is multiplied to the best index score.
        returns:
            best_index: The path index which is best suited for the vehicle to
                navigate with.
        """
        best_index = None
        best_score = float('Inf')
        for i in range(len(paths)):
            # Handle the case of collision-free paths. 只处理无碰撞轨迹
            # print("In selecting "+str(i)) # 显示当前处理轨迹
            if collision_check_array[i]:
                # print("In selecting free path "+str(i)) # 显示当前处理无碰撞轨迹
                # Compute the "distance from centerline" score.
                # The centerline goal is given by goal_state.
                # The exact choice of objective function is up to you.
                # A lower score implies a more suitable path.
                # TODO: INSERT YOUR CODE BETWEEN THE DASHED LINES
                # --------------------------------------------------------------
                path = paths[i]
                score = (goal_state[0]-path[0][-1])*(goal_state[0]-path[0][-1])+(goal_state[1]-path[1][-1])*(goal_state[1]-path[1][-1])
                # --------------------------------------------------------------

                # Compute the "proximity to other colliding paths" score and 与碰撞轨迹的距离
                # add it to the "distance from centerline" score.
                # The exact choice of objective function is up to you.
                for j in range(len(paths)):
                    if j == i:
                        continue
                    else:
                        if not collision_check_array[j]:
                            # TODO: INSERT YOUR CODE BETWEEN THE DASHED LINES
                            # --------------------------------------------------
                            overall_dis = 0.0
                            cur_collision_path = paths[j] # 当前对比轨迹
                            for k in range(len(cur_collision_path[0])): # 所有点？
                                overall_dis = (path[0][k]-cur_collision_path[0][k])*(path[0][k]-cur_collision_path[0][k]) \
                                    +(path[1][k]-cur_collision_path[1][k])*(path[1][k]-cur_collision_path[1][k])
                            score += self._weight * overall_dis
                            # --------------------------------------------------

                            pass

            # Handle the case of colliding paths.
            else:
                score = float('Inf')
                
            # Set the best index to be the path index with the lowest score
            if score < best_score:
                best_score = score
                best_index = i
                
            # print("The best path is "+str(best_index) +' with score '+str(best_score))

        return best_index
