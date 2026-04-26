import numpy as np
import os 
import sys

from .logic.trajectory import define_trayectory_configs, define_trayectory_movements
from .models.scene import Configuration, Scene
from .models.Cspace import CSpace
from .logic.plots import *
from .logic.grid import discretizar_cspace
from .logic.pathfinding import *


def get_start_goal_cell(scene, res):
        qi = scene.conf_init
        qf = scene.conf_final
        
        theta0 = math.radians(qi.theta)
        thetaf = math.radians(qf.theta)

        x0 = qi.point.x
        y0 = qi.point.y

        xf = qf.point.x
        yf = qf.point.y

        start = (int(y0 / res), int(x0 / res))
        goal  = (int(yf / res), int(xf / res))
        return start, goal
    

def cargar_escena( numero_escena):
    """
    Lee el archivo de la escena indicada y guarda el texto en texto_escena.
    """
    # Calculamos la ruta subiendo un nivel de directorio desde este archivo hasta la carpeta 'data'
    # workspace_path = os.path.expanduser('~/proyecto')
    directorio_actual = os.path.dirname(os.path.abspath(__file__))
    print(directorio_actual)
    ruta_archivo = os.path.join(directorio_actual, 'data', f'Escena-Problema{numero_escena}.txt')
    # ruta_archivo = os.path.join(workspace_path, 'data', f'Escena-Problema{numero_escena}.txt')
    workspace_path = os.path.expanduser('~/proyecto')
    ruta_archivo = os.path.join(directorio_actual, '..', 'data', f'Escena-Problema{numero_escena}.txt')
    
    try:
        with open(ruta_archivo, 'r', encoding='utf-8') as archivo:
            return archivo.read()
        # Opcional: imprimir un pedacito para confirmar
        print(f"\n--- Contenido Escena {numero_escena} ---\n{texto_escena}\n---------------------------")
    except FileNotFoundError:
            print(f"No se encontró el archivo: {ruta_archivo}")
    except Exception as e:
            print(f"Error al leer la escena: {e}")

def parse_scene_text(text:str):
    scene = {}
    for i, line in enumerate(text.splitlines()):
        print(line)
        data = line.split(',')
        scene[data[0].lower()] = list(map(float, data[1:]))
    return scene

if __name__ == "__main__":
    numero = int(sys.argv[1])
    txt = cargar_escena(numero)

    scene_raw = parse_scene_text(txt)
    scene = Scene(scene_raw)

    cspace = CSpace(
            scene.robot_geom, 
            scene.obstacles, 
            0
        )
                            
    configure_plot()

    plot_polygon(
    [[0,0], [scene.dimension[0], 0], [*scene.dimension], [0, scene.dimension[1]]], 'y:', thickness=5, 
    labelstr='Frontera W-Space / C-Space'
    )

    plot_polygon(
    cspace.robot_rotated.points,
    color='b--', 
    labelstr=f'Robot (rotado θ={scene.conf_init.theta})'
    )

    for obs in cspace.obstacles_geom:
        plot_polygon(obs.points, 'r-', labelstr=f'Obstáculo {obs.id}')

    for idx, cobs in enumerate(cspace.c_obstacles):
        plot_polygon(cobs, 'g-', labelstr=f'C-Obstáculo {idx + 1}')

    # ======================================
    # 2. Discretizar el espacio de trabajo
    # ======================================

    res = 0.2
    grid, cells = discretizar_cspace(scene, cspace, resolucion=res)

    plot_cell_classification(cells, grid)

    # ================================
    # 3. Calcular ruta - algoritmo A*
    # ================================

    start, goal = get_start_goal_cell(scene, res)
    path = astar(grid, start, goal)
    if path:
        print("Ruta encontrada")
        print(path)

        plot_path(
        path, 
        scene.conf_init.conf[:2],
        scene.conf_final.conf[:2],
        res
        )

        theta0 = math.radians(scene.conf_init.theta)
        plot_robot_orientation(
        scene.conf_init.conf[:2], 
        theta0 
        )

        thetaf = math.radians(scene.conf_final.theta)
        plot_robot_orientation(
            scene.conf_final.conf[:2], 
            thetaf
        )

        

        # ================================
        # 4. Ejecutar trayectoria
        # ================================
        # --> 4.1 Convertir representación matricial a trayectoria
        # discrete_space es una matrix nxm donde M[i, j] = [a, b , c, d, e]  y cada uno de esos a su vez es una coordenada (x, y)
            # c ---------- d
            # |            |
            # |            |
            # |      e     |
            # |            |
            # |            |
            # a ---------- b
        # show()
        
        configs_list = define_trayectory_configs(cells, path, scene.conf_init)

        for c in configs_list:
            print(c)
            
        print("=" * 45)

        movements = define_trayectory_movements(configs_list)

        for m in movements:
            print(m)
            
        