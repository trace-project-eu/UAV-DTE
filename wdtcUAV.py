import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from handleGeo.ConvCoords import ConvCoords


def getWDTC(GFZ, initialPosition, destinationPosition, flightAltitude, hSpeed, vSpeed, useCost, visualize):
    # Generate trajectory
    trajectories = generateTrajectory(GFZ, [initialPosition, destinationPosition], flightAltitude, visualize)
    wgs84path = trajectories[0]
    nedPath = trajectories[1]

    # Calculate distance
    dst = calculateDistance(nedPath)

    # Estimate time
    time =  estimateTime(dst, len(wgs84path), hSpeed, vSpeed, 0)

    # Calculate cost
    cost = calculateCost(time, useCost)

    return [wgs84path, dst, time, cost]



def generateTrajectory(GFZ, locations, flightAltitude, visualize):
    # add zero altitude to all locations
    temp = np.array(locations)
    lcAlts = np.zeros((len(locations), 3))
    lcAlts[:,:-1]  = temp

    # define initialPosition as reference point for the conversions
    convertor = ConvCoords([], [], lcAlts[0].tolist())

    # convert initial position and destination to NED
    nedPath = convertor.wgs84_to_ned(lcAlts.tolist())

    # add intermediate WPs
    if not GFZ:
        for i in range(0, (len(nedPath)-1)*3, 3):
            nedPath = np.insert(nedPath, i+1, nedPath[i], 0)
            nedPath[i+1][2] =  flightAltitude
            nedPath = np.insert(nedPath, i+2, nedPath[i+2], 0)
            nedPath[i+2][2] = flightAltitude
    else:
        # TODO: add intermediate WPs for geo-fenced zones with A*
        # ~~~~~~~~~~~~ temp for visualization development ~~~~~~~~~~~~ #
        for i in range(0, (len(nedPath)-1)*3, 3):
            nedPath = np.insert(nedPath, i+1, nedPath[i], 0)
            nedPath[i+1][2] =  flightAltitude
            nedPath = np.insert(nedPath, i+2, nedPath[i+2], 0)
            nedPath[i+2][2] = flightAltitude
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

    # visualize path
    if visualize:
        # TODO: Visualize trajectory
        print("Trajectory visualization to be implemented soon...")
        # visualizeTrajectory(GFZ, wgs84path)

    # convert ned path to WGS84 coordinates
    wgs84Path = convertor.ned_to_wgs84([nedPath.tolist()])[0]
    return [wgs84Path, nedPath.tolist()]


def visualizeTrajectory(GFZ, wgs84Path):
    # convert trajectory to NED
    convertor = ConvCoords([], [], wgs84Path.tolist()[0])
    nedPath = convertor.wgs84_to_ned(wgs84Path.tolist())

    # add zero altitude to all GFZ vertices
    temp = np.array(GFZ[0])
    GFZ1 = np.zeros((len(GFZ[0]), 3))
    GFZ1[:,:-1]  = temp

    # convert initial position and destination to NED
    nedGfzAlt = convertor.wgs84_to_ned(GFZ1.tolist())
    nedGfz = np.array(nedGfzAlt)[:, 0:2].tolist()

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

    # Plotting the 2D polygon
    poly_x, poly_y = zip(*nedGfz)
    ax1.fill(poly_x, poly_y, alpha=0.35, color='green', label='GFZ Polygon')
    ax1.legend(loc='lower left')

    # 3D subplot
    ax2 = fig.add_subplot(122, projection='3d')

    # Explicitly set axis limits
    ax2.set_xlim(min(xs), max(xs))
    ax2.set_ylim(min(ys), max(ys))
    ax2.set_zlim(min(zs), max(zs))

    # Axis on/off & axis labels
    ax2.axis('off')
    # ax2.set_xticklabels([])
    # ax2.set_yticklabels([])
    # ax2.set_zticklabels([])

    # Plot the 3D trajectory
    ax2.plot3D(xs, ys, zs, 'blue', label='Trajectory')
    ax2.scatter(xs[0], ys[0], zs[0], color='orange', marker='x', label='Initial position')
    ax2.set_title('3D view of UAV trajectory')

    # Create and plot the 3D polygon
    poly3D = [[(x, y, 0) for (x, y) in nedGfz]]
    poly3d = Poly3DCollection(poly3D, alpha=0.35, color='green', label='3D Polygon')
    ax2.add_collection3d(poly3d)

    plt.tight_layout()
    plt.show()



def calculateDistance(nedPath):
    distance = 0
    for i in range(len(nedPath) - 1):
        distance += math.dist(nedPath[i], nedPath[i + 1])

    return distance



def estimateTime(dst, waypointsNumber, horizontalSpeed, verticalSpeed, delayPerStop):
    #   a, b are constants used to regulate the balance between   #
    # horizontal and vertical speeds for the generated trajectory #
    a = 0.85 # coefficient for horizontal speed
    b = 0.15 # coefficient for vertical speed

    #  c, d are constants for the sigmoid function, used to  #
    # calculate a delay for each WP, parametric to the speed #
    c = 5
    d = 20

    # calculate estimated time in minutes (linear time + delay per WP - parametric to the UAV's speed)
    # delay adds a time, parametric to the speed of the UAV for each turn (waypoint)
    # delayPerStop is an additional time for each landing-delivery position of the UAV
    balancedSpeed = a * horizontalSpeed + b * verticalSpeed
    delay = (c * balancedSpeed / (d + abs(balancedSpeed))) * waypointsNumber
    time = dst / (60 * balancedSpeed) + delay + delayPerStop * (waypointsNumber - 1) / 3

    return time



def calculateCost(time, useCost):
    return time * useCost / 60



def generateMatrices(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost):
    # Generate matrices
    distanceMatrix = []
    timeMatrix = []
    for i in range(len(positions)):
        dm = []
        tm = []
        for j in range(len(positions)):
            if i == j:
                dm.append(0)
                tm.append(0)
            else:
                values = getWDTC(GFZ, positions[i], positions[j], flightAltitude, hSpeed, vSpeedMax, useCost, False)
                dm.append(values[1])
                tm.append(values[2])
        distanceMatrix.append(dm)
        timeMatrix.append(tm)

    return [distanceMatrix,timeMatrix]



def getDistanceMatrix(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost):
    return generateMatrices(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost)[0]



def getTimeMatrix(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost):
    return generateMatrices(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost)[1]