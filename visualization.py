import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.plotting import plot_polygon

from handleGeo.ConvCoords import ConvCoords


def visualizeTrajectory(GFZ, wgs84Path):
    # convert trajectory to NED
    convertor = ConvCoords([], [], wgs84Path[0])
    nedPath = convertor.wgs84_to_ned(wgs84Path)

    xs = nedPath[:, 0].tolist()
    ys = nedPath[:, 1].tolist()
    zs = nedPath[:, 2].tolist()

    # Create a figure with subplots
    fig = plt.figure(figsize=(12, 6))

    # 2D subplot
    ax1 = fig.add_subplot(121)
    ax1.axis('off')
    ax1.scatter(xs[0], ys[0], color='orange', marker='x', label='Initial position')
    ax1.plot(xs, ys, 'blue', label='Trajectory')
    ax1.set_title('2D view of UAV trajectory')

    # 3D subplot
    ax2 = fig.add_subplot(122, projection='3d')

    # Axis on/off & axis labels
    ax2.axis('off')
    # ax2.set_xticklabels([])
    # ax2.set_yticklabels([])
    # ax2.set_zticklabels([])

    # Plot the 3D trajectory
    ax2.plot3D(xs, ys, zs, 'blue', label='Trajectory')
    ax2.scatter(xs[0], ys[0], zs[0], color='orange', marker='x', label='Initial position')
    ax2.set_title('3D view of UAV trajectory')


    if GFZ:
        # # add zero altitude to all GFZ vertices
        # temp = np.array(GFZ[0])
        # GFZ1 = np.zeros((len(GFZ[0]), 3))
        # GFZ1[:,:-1]  = temp
        #
        # # convert GFZ to NED
        # nedGfzAlt = convertor.wgs84_to_ned(GFZ1.tolist())
        # nedGfz = np.array(nedGfzAlt)[:, 0:2].tolist()
        #
        # # Plotting the 2D polygon
        # poly_x, poly_y = zip(*nedGfz)
        # ax1.fill(poly_x, poly_y, alpha=0.35, color='green', label='GFZ Polygon')
        # ax1.legend(loc='lower left')
        #
        # # Create and plot the 3D polygon
        # poly3D = [[(x, y, 0) for (x, y) in nedGfz]]
        # poly3d = Poly3DCollection(poly3D, alpha=0.35, color='green', label='3D Polygon')
        # ax2.add_collection3d(poly3d)
        plot_polygon(GFZ[0], convertor, ax1, ax2, 'green', 'Geofence Zone')

        if len(GFZ) > 1:
            for i in range(1, len(GFZ)):
                if i==1:
                    plot_polygon(GFZ[i], convertor, ax1, ax2, 'red', 'No-Go-Zone')
                else:
                    plot_polygon(GFZ[i], convertor, ax1, ax2, 'red', '')

    plt.tight_layout()
    plt.show()

def plot_polygon(poly, convertor, ax1, ax2, color, label):
    temp = np.array(poly)
    GFZ1 = np.zeros((len(poly), 3))
    GFZ1[:, :-1] = temp

    # convert GFZ to NED
    nedGfzAlt = convertor.wgs84_to_ned(GFZ1.tolist())
    nedGfz = np.array(nedGfzAlt)[:, 0:2].tolist()

    # Plotting the 2D polygon
    poly_x, poly_y = zip(*nedGfz)
    if label:
        ax1.fill(poly_x, poly_y, alpha=0.35, color=color, label=label)
    else:
        ax1.fill(poly_x, poly_y, alpha=0.35, color=color)
    ax1.legend(loc='lower left')

    # Create and plot the 3D polygon
    poly3D = [[(x, y, 0) for (x, y) in nedGfz]]
    if label:
        poly3d = Poly3DCollection(poly3D, alpha=0.35, color=color, label=label)
    else:
        poly3d = Poly3DCollection(poly3D, alpha=0.35, color=color)
    ax2.add_collection3d(poly3d)