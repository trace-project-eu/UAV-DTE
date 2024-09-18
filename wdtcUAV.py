import math
from handleGeo.ConvCoords import ConvCoords
import numpy as np

from handleGeo.coordinates.WGS84 import distance


def getWDTC(GFZ, initialPosition, destinationPosition, flightAltitude, hSpeed, vSpeed, useCost, visualize):

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    #                                              Generate Trajectory
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    trajectories = generateTrajectory(GFZ, [initialPosition, destinationPosition], flightAltitude, visualize)
    wgs84path = trajectories[0]
    nedPath = trajectories[1]
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #



    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    #                                               Calculate Distance
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    dst = calculateDistance(nedPath)
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    #                                               Estimate Time
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    #   a, b are constants used to regulate the balance between   #
    # horizontal and vertical speeds for the generated trajectory #
    a = 0.85 # coefficient for horizontal speed
    b = 0.15 # coefficient for vertical speed

    #  c, d are constants for the sigmoid function, used to  #
    # calculate a delay for each WP, parametric to the speed #
    c = 5
    d = 20

    # calculate estimated time in minutes (linear time + delay per WP - parametric to the UAV's speed)
    balancedSpeed = a * hSpeed + b * vSpeed
    delay = c * balancedSpeed / (d + abs(balancedSpeed))
    time = dst / (60 * balancedSpeed) + delay
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    #                                              Calculate Cost
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    cost = time * useCost / 60
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #


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
        print("Support of geo-fenced zones to be implemented soon...")

    # visualize path
    if visualize:
        # TODO: Visualize trajectory
        print("Trajectory visualization to be implemented soon...")

    # convert ned path to WGS84 coordinates
    wgs84Path = convertor.ned_to_wgs84([nedPath.tolist()])[0]
    return [wgs84Path, nedPath.tolist()]



def visualizeTrajectory():
    # TODO: implement trajectory visualization
    print()



def calculateDistance(nedPath):
    distance = 0
    for i in range(len(nedPath) - 1):
        distance += math.dist(nedPath[i], nedPath[i + 1])

    return distance



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