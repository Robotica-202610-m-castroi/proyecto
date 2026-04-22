import numpy as np

def discretizar_cspace(scene, cspace, resolucion=0.2):
    """
    Divide el espacio en una grilla y clasifica cada celda.
    """
    width, height = scene.dimension
    
    cols = int(width / resolucion)
    rows = int(height / resolucion)

    grid = np.empty((rows, cols), dtype=object)
    cells = np.empty((rows, cols), dtype=object)

    def punto_en_poligono(x, y, poly):
        """Algoritmo ray casting"""
        inside = False
        n = len(poly)
        px, py = x, y

        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i+1) % n]

            if ((y1 > py) != (y2 > py)) and \
               (px < (x2 - x1) * (py - y1) / (y2 - y1 + 1e-9) + x1):
                inside = not inside

        return inside

    for i in range(rows):
        for j in range(cols):
            
            # Coordenadas de la celda
            x_min = j * resolucion
            y_min = i * resolucion
            x_max = x_min + resolucion
            y_max = y_min + resolucion

            # Esquinas de la celda
            cell = [
                (x_min, y_min),
                (x_max, y_min),
                (x_min, y_max),
                (x_max, y_max)
            ]
            cells[i, j] = cell

            # Verificar ocupación
            puntos_dentro = 0

            for px, py in cell:
                for cobs in cspace.c_obstacles:
                    if punto_en_poligono(px, py, cobs):
                        puntos_dentro += 1
                        break

            if puntos_dentro == 4:
                grid[i, j] = "black"
            elif puntos_dentro > 0:
                grid[i, j] = "gray"
            else:
                grid[i, j] = "white"

    return grid, cells