import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.plotting import plot_polygon
from handleGeo.ConvCoords import ConvCoords


def visualizeTrajectory(gfz, wgs84_path):
    """
    Visualizes a UAV trajectory in 2D and 3D views, with optional geofence zones.

    Args:
        gfz (list): List of geo-fence zone and no-go zones in WGS84 coordinates.
        wgs84_path (list): UAV trajectory in WGS84 coordinates.

    Returns:
        None: Displays 2D and 3D visualizations of the trajectory.
    """
    try:
        # Convert trajectory to NED coordinates
        convertor = ConvCoords([], [], wgs84_path[0])
        ned_path = convertor.wgs84_to_ned(wgs84_path)

        # Extract X, Y, Z coordinates for plotting
        xs, ys, zs = ned_path[:, 0].tolist(), ned_path[:, 1].tolist(), ned_path[:, 2].tolist()

        # Create a figure with 2D and 3D subplots
        fig = plt.figure(figsize=(12, 6))

        # 2D Subplot
        ax1 = fig.add_subplot(121)
        ax1.axis('off')
        ax1.scatter(xs[0], ys[0], color='orange', marker='x', label='Initial position')
        ax1.plot(xs, ys, 'blue', label='Trajectory')
        ax1.set_title('2D view of UAV trajectory')

        # 3D Subplot
        ax2 = fig.add_subplot(122, projection='3d')
        ax2.axis('off')
        ax2.plot3D(xs, ys, zs, 'blue', label='Trajectory')
        ax2.scatter(xs[0], ys[0], zs[0], color='orange', marker='x', label='Initial position')
        ax2.set_title('3D view of UAV trajectory')

        # Plot geofence and no-go zones if provided
        if gfz:
            plot_polygon(gfz[0], convertor, ax1, ax2, 'green', 'Geofence Zone')

            for i in range(1, len(gfz)):
                if i == 1:
                    plot_polygon(gfz[i], convertor, ax1, ax2, 'red', 'No-Go-Zone')
                else:
                    plot_polygon(gfz[i], convertor, ax1, ax2, 'red', '')

        # Adjust layout and display the plot
        plt.tight_layout()
        plt.show()

    except Exception as e:
        print(f"Error in visualizing trajectory: {e}")


def plot_polygon(poly, convertor, ax1, ax2, color, label):
    """
    Plots a polygon in 2D and 3D views.

    Args:
        poly (list): Polygon vertices in WGS84 coordinates.
        convertor (ConvCoords): ConvCoords instance containing the reference point to transform WGS84 to NED.
        ax1 (matplotlib.Axes): 2D plotting axis.
        ax2 (matplotlib.Axes): 3D plotting axis.
        color (str): Color of the polygon.
        label (str): Label for the polygon in the plot legend.

    Returns:
        None: Adds polygon visualization to the provided axes.
    """
    try:
        # Convert polygon vertices to NED coordinates
        temp = np.array(poly)
        gfz_ned = np.zeros((len(poly), 3))
        gfz_ned[:, :-1] = temp
        ned_gfz_alt = convertor.wgs84_to_ned(gfz_ned.tolist())
        ned_gfz = np.array(ned_gfz_alt)[:, 0:2].tolist()

        # Plot the 2D polygon
        poly_x, poly_y = zip(*ned_gfz)
        if label:
            ax1.fill(poly_x, poly_y, alpha=0.35, color=color, label=label)
        else:
            ax1.fill(poly_x, poly_y, alpha=0.35, color=color)
        ax1.legend(loc='lower left')

        # Plot the 3D polygon
        poly3D = [[(x, y, 0) for (x, y) in ned_gfz]]
        poly3d = Poly3DCollection(poly3D, alpha=0.35, color=color, label=label if label else None)
        ax2.add_collection3d(poly3d)

    except Exception as e:
        print(f"Error in plotting polygon: {e}")