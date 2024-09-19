import math
import numpy as np

from extremitypathfinder import PolygonEnvironment
from handleGeo.ConvCoords import ConvCoords
from visualization import *


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
        environment = PolygonEnvironment()

        # add zero altitude to all GFZ vertices
        temp = np.array(GFZ[0])
        GFZ1 = np.zeros((len(GFZ[0]), 3))
        GFZ1[:, :-1] = temp

        # convert initial position and destination to NED
        nedGfzAlt = convertor.wgs84_to_ned(GFZ1.tolist())
        polygon = np.array(nedGfzAlt)[:, 0:2].tolist()

        list_of_holes = []
        # for i in range(1, (len(GFZ))):
        #     # add zero altitude to all GFZ vertices
        #     temp = np.array(GFZ[i])
        #     NFZ = np.zeros((len(GFZ[i]), 3))
        #     NFZ[:, :-1] = temp
        #     nedNfzAlt = convertor.wgs84_to_ned(GFZ1.tolist())
        #     list_of_holes.append(np.array(nedNfzAlt)[:, 0:2].tolist())

        environment.store(polygon, list_of_holes, validate=False)

        nedPathGFZ = []
        for i in range(0, len(nedPath)-1):
            start_coordinates = tuple(nedPath[i][0:2])
            goal_coordinates = tuple(nedPath[i+1][0:2])
            intermediate_path = environment.find_shortest_path(start_coordinates, goal_coordinates)[0]

            nedPathGFZ.append(nedPath[i])
            for j in range(0, len(intermediate_path)):
                nedPathGFZ.append(np.insert(np.array(intermediate_path[j]), 2, flightAltitude, 0))
            nedPathGFZ.append(nedPath[i+1])

        nedPath = np.array(nedPathGFZ)

    # convert ned path to WGS84 coordinates
    wgs84Path = convertor.ned_to_wgs84([nedPath.tolist()])[0]
    # visualize path
    if visualize:
        visualizeTrajectory(GFZ, wgs84Path)

    return [wgs84Path, nedPath.tolist()]



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