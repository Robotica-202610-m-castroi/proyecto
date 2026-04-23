import numpy as np

def discretizar_cspace(scene, cspace, resolucion=0.2):

    width, height = scene.dimension
    
    cols = int(width / resolucion)
    rows = int(height / resolucion)

    grid = np.empty((rows, cols), dtype=object)
    cells = np.empty((rows, cols), dtype=object)

    def punto_en_poligono(x, y, poly):
        inside = False
        n = len(poly)

        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i+1) % n]

            if ((y1 > y) != (y2 > y)) and \
               (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-9) + x1):
                inside = not inside

        return inside

    for i in range(rows):
        for j in range(cols):

            x_min = j * resolucion
            y_min = i * resolucion
            x_max = x_min + resolucion
            y_max = y_min + resolucion

            # esquinas
            a = (x_min, y_min)
            b = (x_max, y_min)
            c = (x_min, y_max)
            d = (x_max, y_max)

            cell = [a, b, c, d]
            cells[i, j] = cell

            # 🔥 1. frontera = gris
            if (
                x_min <= 0 or y_min <= 0 or
                x_max >= width or y_max >= height
            ):
                grid[i, j] = "gray"
                continue

            puntos_dentro = 0

            for px, py in cell:
                for cobs in cspace.c_obstacles:
                    if punto_en_poligono(px, py, cobs):
                        puntos_dentro += 1
                        break

            # 🔥 2. clasificación robusta
            if puntos_dentro >= 2:
                grid[i, j] = "black"   # obstáculo grande o unión
            elif puntos_dentro == 1:
                grid[i, j] = "gray"
            else:
                grid[i, j] = "white"

    return grid, cells