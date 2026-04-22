import numpy as np

class Movement:
    
    def __init__(self, is_rotation, dx, dy, da) -> None:
        self.is_rotation = is_rotation
        self.dx = dx
        self.dy = dy
        self.da = da
    
    def __repr__(self) -> str:
        return f"Displacement(rotation={self.is_rotation}, dx={self.dx}, dy={self.dy}, da={self.da})"
    
    def __str__(self) -> str:
        return f"Displacement(rotation={self.is_rotation}, dx={self.dx:.2f}, dy={self.dy:.2f}, da={self.da:.2f})"



# TODO: clean up, point might not be necessary
class Point:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y
    
    def distances(self, p):
        dx = self.x - p.x
        dy = self.y - p.y
        return dx, dy
    
    def to_list(self):
        return [self.x, self.y]
    
    def __repr__(self):
        return f"Point(x={self.x}, y={self.y})"
    
    def __str__(self):
        return f"({self.x}, {self.y})"

class Configuration:
    def __init__(self, x, y, theta) -> None:
        self.point = Point(x, y)
        self.theta = theta
        self.conf = np.array([x, y, theta])
    
    def __repr__(self):
        return f"Configuration(x={self.point.x}, y={self.point.y}, theta={self.theta})"
    
    def __str__(self):
        return f"Configuration({self.point.x}, {self.point.y}, {self.theta})"
        
class Obstacle:
    def __init__(self, id, data) -> None:
        self.id = id
        self.pt1 = Point(*data[0])
        self.pt2 = Point(*data[1])
        
        self.pt_list = self.__find_rect_points()
        
    def __find_rect_points(self):
        dx, dy = self.pt2.distances(self.pt1)
        pt_br = [self.pt1.x + dx, self.pt1.y]
        pt_tf = [self.pt1.x, self.pt1.y + dy]
        
        return [self.pt1.to_list(), pt_br, self.pt2.to_list(), pt_tf]
    
    def __repr__(self):
        return f"Obstacle(id={self.id}, pt1=({self.pt1.x}, {self.pt1.y}), pt2=({self.pt2.x}, {self.pt2.y}))"
    
    def __str__(self):
        return f"Obstacle {self.id}: ({self.pt1.x}, {self.pt1.y}) -> ({self.pt2.x}, {self.pt2.y})"

class Scene:
    def __init__(self, raw_scene) -> None:
        self.dimension = tuple(raw_scene['dimensiones'])
        self.conf_init = Configuration(*raw_scene['q0'])
        self.conf_final = Configuration(*raw_scene['qf'])
        self.dist_front = raw_scene['dfrente'][0]
        self.dist_right = raw_scene['dderecha'][0]
        
        self.obstacles = []
        
        for i in range(int(raw_scene['obstaculos'][0])):
            pt1 = raw_scene[f'obstaculo{i+1}_pto1']
            pt2 = raw_scene[f'obstaculo{i+1}_pto2']
            
            self.obstacles.append(Obstacle(i+1, [pt1, pt2]))
        self.robot_geom =  self.calculate_robot_geometry_at_origin(self.conf_init.point)
         
    def calculate_robot_geometry_at_origin(self, pt: Point):
        dist = .3 / 2

        geom_pts =  np.array([[pt.x - dist, pt.y - dist], [pt.x + dist, pt.y - dist], [pt.x + dist, pt.y + dist], [pt.x - dist, pt.y + dist]])
        
        return [np.array(pt) - np.array(*geom_pts[0]) for pt in geom_pts]
 
    
    def __str__(self):
        obstacles_str = '\n  '.join(f"Obstacle {obs.id}: ({obs.pt1.x}, {obs.pt1.y}) -> ({obs.pt2.x}, {obs.pt2.y})" for obs in self.obstacles)
        return f"Scene(dimension={self.dimension}, q0={self.conf_init}, qf={self.conf_final}, dfrente={self.dist_front}, dderecha={self.dist_right})\nObstacles:\n  {obstacles_str}"
    
    def __repr__(self):
        return f"Scene(dimension={self.dimension}, conf_init={self.conf_init}, conf_final={self.conf_final}, dist_front={self.dist_front}, dist_right={self.dist_right}, obstacles={len(self.obstacles)})"
