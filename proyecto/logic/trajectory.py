import numpy as np
import math 

from ..models.scene import Configuration, Movement

def define_trayectory_configs(discrete_space, route: list[list[int]], qf: Configuration):
    configurations: list[Configuration] = []
    

    s = 0
    while s < len(route) - 1:
        current_cell, next_cell = route[s], route[s + 1]
        
        current_cell_center = np.array(list(map( lambda x: round(x, 2), discrete_space[*current_cell][0])))
        next_cell_center = np.array(list(map( lambda x: round(x, 2), discrete_space[*next_cell][0])))
        
        diff = np.sign(current_cell_center - next_cell_center)
        theta = math.atan2(*diff)
        
        
        
        if s == 0:
            prev_diff = diff

        if np.array_equal(diff, prev_diff):
            configurations.append(Configuration(*current_cell_center, 0))
        else:
            theta = math.degrees(math.asin((diff - prev_diff)[0]))
            
            configurations.append(Configuration(*current_cell_center, 0))
            configurations.append(Configuration(*current_cell_center, theta))
            
            
        if s == len(route) - 2:
            configurations.append(Configuration(*next_cell_center, 0))
            theta = math.degrees(math.asin((np.array([0, 1]) - prev_diff)[0]))
            configurations.append(Configuration(*next_cell_center, theta))
            
        prev_diff = diff
        s += 1
        
    return configurations

def define_trayectory_movements(configurations: list[Configuration]):
    
    q_c = configurations.pop(0)
    
    movements: list[Movement] = []
    
    t_dx = 0
    t_dy = 0
    
    for q_n in configurations:
        dx, dy, d0 = q_n.conf - q_c.conf
        t_dx += dx
        t_dy += dy
        if movements:
            prev_was_rotation = movements[-1].is_rotation 
            if prev_was_rotation:
                d0 = 0
        movements.append(Movement(d0 != 0, dx, dy, d0))
        q_c = q_n
        
    print(f"total expected displacement: x={t_dx}, y={t_dy}")
    return movements