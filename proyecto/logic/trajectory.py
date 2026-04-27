import numpy as np
import math 

from ..models.scene import Configuration, Movement

def signed_angle(a: list, b: list):
    a = np.array(a)
    b = np.array(b)
    cross = np.cross(a, b)
    dot   = np.dot(a, b)
    return np.degrees(np.arctan2(cross, dot))[-1]

def dir_vect(theta):
    rad = np.radians(theta)
    return np.array([np.cos(rad), np.sin(rad)]).astype(int)

def define_trayectory_configs(discrete_space, route: list[list[int]], qi: Configuration):
    configurations: list[Configuration] = []

    s = 0
    while s < len(route) - 1:
        current_cell, next_cell = route[s], route[s + 1]
        
        current_cell_center = np.array(list(map( lambda x: round(x, 2), discrete_space[*current_cell][0])))
        next_cell_center = np.array(list(map( lambda x: round(x, 2), discrete_space[*next_cell][0])))
        
        diff = np.append(np.sign(next_cell_center - current_cell_center), 0)
                
        if s == 0:
            prev_diff = np.append(dir_vect(qi.theta), 0)
            prev_theta = qi.theta
        else:
            prev_theta = configurations[-1].theta   
            
            
        if np.array_equal(diff, prev_diff):
            configurations.append(Configuration(*current_cell_center, prev_theta))
        else:

            theta = signed_angle(prev_diff, diff)
            configurations.append(Configuration(*current_cell_center, prev_theta))
            configurations.append(Configuration(*current_cell_center, theta + prev_theta))
            
            
        if s == len(route) - 2:
            theta = signed_angle(prev_diff, diff)
            configurations.append(Configuration(*next_cell_center, prev_theta))
            configurations.append(Configuration(*next_cell_center, theta + prev_theta))
            
        prev_diff = diff
        s += 1
        
    return configurations

def define_trayectory_movements(configurations: list[Configuration]):
    
    q_c = configurations[0]
    
    movements: list[Movement] = []
    
    t_dx = 0
    t_dy = 0
    
    for q_n in configurations[1:]:
        dx, dy, d0 = q_n.conf - q_c.conf
        t_dx += abs(dx)
        t_dy += abs(dy)
        if movements:
            prev_was_rotation = movements[-1].is_rotation 
            if prev_was_rotation:
                d0 = 0
        movements.append(Movement(d0 != 0, dx, dy, d0))
        q_c = q_n
        
    print(f"total expected displacement: x={t_dx}, y={t_dy}")
    return movements