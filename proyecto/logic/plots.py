# import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import math
import datetime

# matplotlib.use('SVG')

def plot_unit_circle(vectors):
    ax = plt.gca()
    
    for normal_data in vectors:
        normal, source, idx = normal_data

        if source == 'r':
            xytext = (-10, 10)
            labl = 'r'
            source = 'blue' 
        else:
            xytext = (10, -10)
            labl = 'o'
            source = 'red' 

        ax.arrow(0, 0, normal[0], normal[1],
                 head_width=0.1, head_length=0.1,
                 fc=source, ec=source)

        ax.annotate(f'{labl}{idx+1}',
                    xy=(normal[0], normal[1]),
                    xytext=xytext,
                    textcoords='offset points')
    
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Normal Vectors')


def plot_polygon(geometry, color='b-', thickness=2, labelstr=''):
    ax = plt.gca()
    
    for i, p in enumerate(geometry):

        x0, y0 = p

        if i == len(geometry) - 1:
            x1, y1 = geometry[0]
        else:
            x1, y1 = geometry[i+1]

        ax.plot([x0, x1], [y0, y1],
                 color,
                 linewidth=thickness,
                 label=labelstr if i == 0 else "")

        ax.plot(x0, y0, color[0] + 'o')


def plot_discrete_w_space(discrete_w_space):
    ax = plt.gca()

    for i in discrete_w_space:
        for j in i:

            for pt in j:
                ax.plot(pt[0], pt[1], 'kx')

            ax.plot(np.array([j[0], j[1]])[:,0],
                     np.array([j[0], j[1]])[:,1], 'k--')

            ax.plot(np.array([j[0], j[2]])[:,0],
                     np.array([j[0], j[2]])[:,1], 'k--')

            ax.plot(np.array([j[1], j[3]])[:,0],
                     np.array([j[1], j[3]])[:,1], 'k--')

            ax.plot(np.array([j[2], j[3]])[:,0],
                     np.array([j[2], j[3]])[:,1], 'k--')


def plot_cell_classification(grid, classes):
    ax = plt.gca()

    rows, cols = grid.shape
    plotted_labels = set()
    for i in range(rows):
        for j in range(cols):

            cell = grid[i, j]

            a, b, c, d = cell

            xs = [a[0], b[0], d[0], c[0]]
            ys = [a[1], b[1], d[1], c[1]]

            if classes[i, j] == "white":
                color = "white"
                lbl = "libre"

            elif classes[i, j] == "black":
                color = "black"
                lbl = "ocupada"

            else:
                color = "lightgray"
                lbl = "semi-libre"

            ax.fill(
                xs,
                ys,
                color=color,
                edgecolor='black',
                label= lbl if lbl not in plotted_labels else "",
                alpha=0.9
            )
            
            plotted_labels.add(lbl)


def plot_path(path, init_pt, end_pt, res):
    
    for p in path:
        y, x = p

        x_min = x * res
        y_min = y * res

        cell_xs = [x_min, x_min+res, x_min+res, x_min]
        cell_ys = [y_min, y_min, y_min+res, y_min+res]

        plt.fill(cell_xs, cell_ys, color='red', alpha=0.5)

    # inicio
    plot_config_pt(init_pt, "Inicio", 'bo')

    # final
    plot_config_pt(end_pt, "Final", 'go')
        
def plot_robot_orientation(init_pt, theta):
    dx0 = 0.3 * math.cos(theta)
    dy0 = 0.3 * math.sin(theta)
    plt.arrow(init_pt[0], init_pt[1], dx0, dy0, color='blue', head_width=0.1)

def configure_plot():
        fig, ax = plt.subplots(figsize=(10, 10))
        
        ax.set_aspect('equal')
        ax.grid(True, which='both')
        ax.axhline(y=0, color='k')
        ax.axvline(x=0, color='k')
        
        return fig, ax


def plot_config_pt(pt, label, icon):
        ax = plt.gca()
        x, y = pt
        ax.plot(x, y, icon, label=label)

def save():
    fig = plt.gcf()
    axes = fig.get_axes()

    
    for ax in axes:
        if ax.get_legend_handles_labels()[0]: 
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.10), ncol=3)
    
    plt.tight_layout()
    
    plt.savefig(f'../planificacion_{datetime.datetime.now()}.png')

def show():
    fig = plt.gcf()
    axes = fig.get_axes()

    
    for ax in axes:
        if ax.get_legend_handles_labels()[0]: 
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.10), ncol=3)
    
    plt.tight_layout()
    plt.show(block=False)