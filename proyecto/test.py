import numpy as np

from .logic.trajectory import define_trayectory_configs, define_trayectory_movements
from .models.scene import Configuration

discrete_space = np.load('proyecto/discrete_space_example.npy', allow_pickle=True)
route = [[2, 1], [2, 2], [2, 3], [2, 4], [2, 5], [3, 5], [3, 6], [4, 7], [3, 7], [3, 6]]
configs_list = define_trayectory_configs(discrete_space, route, Configuration(3, 7, 0))

print(route, len(route))
print("Configurations:")
for c in configs_list:
    print(c)
print("-"*45)


movements = define_trayectory_movements(configs_list)

for m in movements:
    print(m)