import numpy as np
from math import cos, sin, pi

from .scene import Obstacle

from ..logic.plots import *

class Geometry:
    
    def __init__(self, pt_list, id, is_obstacle=True) -> None:
        self.sides = len(pt_list)
        self.points = np.array(pt_list)
        self.id = id
        self.is_obstcle = is_obstacle
        self.edges = []
        self.normals = []
        
        self.edges = np.array(self.__calculate_edge())
        self.normals = np.array(self.__calculate_normal())
    
    def __calculate_edge(self):
        return [self.points[idx+1] - self.points[idx] if idx < len(self.points) - 1 else self.points[0] - self.points[idx] for idx in range(len(self.points))]

    def __calculate_normal(self):
        base = [np.flip(pt) for pt in self.edges]
        if not self.is_obstcle:
            # -dy, dx
            return [np.array([-pt[0], pt[1]]) / np.sqrt(np.sum(pt**2))for pt in base]
        else:
            # dy, -dx
            return [np.array([pt[0], -pt[1]]) / np.sqrt(np.sum(pt**2))for pt in base]
    
    def __repr__(self):
        return f"Geometry(id={self.id}, sides={self.sides}, is_obstacle={self.is_obstcle}, points={self.points})"
    
    def __str__(self):
        return f"Geometry(id={self.id}, sides={self.sides}, is_obstacle={self.is_obstcle})"

class CSpace:
    
    def __init__(self, robot: list, obstacles: list[Obstacle], theta: float):
        self.robot = robot
        self.obstacles = obstacles
        self.c_obstacles = []
        self.current_theta = theta
    
        self.robot_geom = Geometry(self.robot, 0, False)
        
        self.obstacles_geom: list[Geometry] = []
        for obs in self.obstacles:
            self.obstacles_geom.append(Geometry(obs.pt_list, obs.id, True))

        self.robot_rotated = self.define_rotation(self.current_theta)
        self.create_configuration_space()
        
    
    def define_rotation(self, theta):
        rot_matrix = np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)]
        ])
        
        pts = []
        for robot_pt in self.robot_geom.points:
            robot_arra = np.array(robot_pt)
            pts.append(np.matmul(rot_matrix, robot_arra.reshape(robot_arra.shape[::-1])))

        return Geometry(pts, 0, False)
    
    def create_configuration_space(self):
        for obs in self.obstacles_geom:
            
            grouped = self.__normal_vectors(self.robot_rotated.normals, obs.normals)
            contacts = self.__contacts(grouped, obs)

            # print('\nVertices de C-Obstáculo')
            pts = []
            for contact in contacts:
                pts.append(contact)
                # print(contact)
            
            self.c_obstacles.append(pts)

    
    
    def __normal_vectors(self, robot_normal, obstacle_normal):
        normals = []
        for i, normal in enumerate(robot_normal):
            normals.append((normal, 'r', i))
            
        for i, normal in enumerate(obstacle_normal):
            normals.append((normal, 'o', i))

        
        normals.sort(key=lambda x: np.arctan2(x[0][1], x[0][0]) % (2 * pi))
        
        # fig, ax_left, ax_right = configure_plot()
        # plot_unit_circle(normals, ax_left)
        # show()
        
        grouped_normals = []
        current_group = [normals[0]]
        
        for i in range(1, len(normals)):
            angle_prev = np.arctan2(normals[i-1][0][1], normals[i-1][0][0]) % (2 * pi)
            angle_curr = np.arctan2(normals[i][0][1], normals[i][0][0]) % (2 * pi)
            
            if np.isclose(angle_prev, angle_curr):
                current_group.append(normals[i])
            else:
                grouped_normals.append(current_group if len(current_group) > 1 else current_group[0])
                current_group = [normals[i]]
        
        grouped_normals.append(current_group if len(current_group) > 1 else current_group[0])

        return grouped_normals
    
    def __contacts(self, grouped_normals, obstacle_geom: Geometry):
        c_obstacle = []
        # print(grouped_normals)
        # print('\nTipos de contacto y sus aristas:')
        for idx, curr_normal in enumerate(grouped_normals):
            if type(curr_normal) == tuple:
                _, normal_type, normal_idx = curr_normal
                next = grouped_normals[(idx+1) % len(grouped_normals)]
                prev = grouped_normals[(idx-1) % len(grouped_normals)]
            
                if type(next) == tuple:
                    _, next_type, next_idx = next
                else:
                    # caso: el siguiente es colineal. escoger normal que niege condición
                    _, next_type, next_idx = tuple(filter(lambda x: x[1] != normal_type, next))[0]

                if type(prev) == tuple:
                    _, prev_type, _ = prev
                else:
                    # caso: el siguiente es colineal. escoger normal que niege condición
                    _, prev_type, _ = tuple(filter(lambda x: x[1] != normal_type, prev))[0]
            
                if normal_type == 'r' and next_type == 'o' and prev_type == 'o':
                    # tipo a
                    prime = (normal_idx + 1) % len(self.robot_rotated.points)
                    # print(f'tipo A {normal_idx+1},{next_idx+1} | b_{next_idx+1}-a_{normal_idx+1} , b_{next_idx+1}-a_{prime+1}')
                    c_obstacle.extend([
                        obstacle_geom.points[next_idx] - self.robot_rotated.points[normal_idx], 
                        obstacle_geom.points[next_idx] - self.robot_rotated.points[prime]
                    ])

            
                if normal_type == 'o' and next_type == 'r' and prev_type == 'r':
                    # tipo b
                    prime = (normal_idx + 1) % len(obstacle_geom.points)
                    # print(f'tipo B {next_idx+1},{normal_idx+1} | b_{normal_idx+1}-a_{next_idx+1} , b_{prime+1}-a_{next_idx+1}')
                    
                    c_obstacle.extend([
                        obstacle_geom.points[normal_idx] - self.robot_rotated.points[next_idx], 
                        obstacle_geom.points[prime] - self.robot_rotated.points[next_idx]
                    ])
            else:
                # tipo c
                (_, _, normal_idx), (_, _, next_idx) = curr_normal
                
                rprime = (normal_idx + 1) % len(self.robot_rotated.points)
                oprime = (next_idx + 1) % len(obstacle_geom.points)
                
                # print(f'tipo C {normal_idx+1},{next_idx+1} | b_{next_idx+1}-a_{normal_idx+1}, b_{next_idx+1}-a_{rprime+1}, b_{oprime+1}-a_{normal_idx+1}, b_{oprime+1}-a_{rprime+1}')

                # print(
                #     obstacle_geom.points[next_idx] , self.robot_rotated.points[normal_idx], '||',
                #     obstacle_geom.points[next_idx] , self.robot_rotated.points[rprime], '||',
                #     obstacle_geom.points[oprime] , self.robot_rotated.points[normal_idx], '||',
                #     obstacle_geom.points[oprime] , self.robot_rotated.points[rprime],  '||',
                # )
                # input()
                # print(
                #     obstacle_geom.points[next_idx] - self.robot_rotated.points[normal_idx], '||',
                #     obstacle_geom.points[next_idx] - self.robot_rotated.points[rprime], '||',
                #     obstacle_geom.points[oprime] - self.robot_rotated.points[normal_idx], '||',
                #     obstacle_geom.points[oprime] - self.robot_rotated.points[rprime],  '||',
                # )
                # input()
                c_obstacle.extend([
                    obstacle_geom.points[next_idx] - self.robot_rotated.points[normal_idx],
                    obstacle_geom.points[next_idx] - self.robot_rotated.points[rprime],
                    obstacle_geom.points[oprime] - self.robot_rotated.points[normal_idx],
                    obstacle_geom.points[oprime] - self.robot_rotated.points[rprime] 
                ])
                
        return c_obstacle
    
    def __repr__(self):
        return f"CSpace(robot_points={len(self.robot)}, obstacles={len(self.obstacles)}, theta={self.current_theta})"
    
    def __str__(self):
        return f"CSpace with {len(self.obstacles)} obstacles, current_theta={self.current_theta}"