import math
from handleGeo.ConvCoords import ConvCoords
import numpy as np


def getWDTC(GFZ, initialPosition, destinationPosition, flightAltitude, hSpeed, vSpeed, useCost, visualize):

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    #                                              Generate Trajectory
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    # add altitude to the initial and final positions
    initialPosition.append(0)
    destinationPosition.append(0)

    # define initialPosition as reference point for the conversions
    convertor = ConvCoords([], [], initialPosition)

    # convert initial position and destination to NED
    nedInitialPosition = convertor.conv_wgs84_to_ned([initialPosition])
    nedDestinationPosition = convertor.conv_wgs84_to_ned([destinationPosition])

    # add intermediate WPs
    if not GFZ:
        nedPath = np.array([nedInitialPosition[0],
                                 np.append(nedInitialPosition[0][0:2], [flightAltitude]),
                                 np.append(nedDestinationPosition[0][0:2], [flightAltitude]),
                                 nedDestinationPosition[0]])
    else:
        # TODO: add intermediate WPs for geo-fenced zones with A*
        print("Support of geo-fenced zones to be implemented soon...")

    # visualize path
    if visualize:
        # TODO: Visualize trajectory
        print("Trajectory visualization to be implemented soon...")

    # convert ned path to WGS84 coordinates
    wgs84path = convertor.ned_to_wgs84([nedPath.tolist()])
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #



    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    #                                               Calculate Distance
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    distance = 0
    for i in range(len(nedPath[0]) - 1):
        distance += math.dist(nedPath[i], nedPath[i + 1])
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
    time = distance / (60 * balancedSpeed) + delay
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #



    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    #                                              Calculate Cost
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    cost = time * useCost / 60
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

    initialPosition.pop()
    destinationPosition.pop()


    return [wgs84path[0], distance, time, cost]


def getDistanceMatrix(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost):
    # Generate distance matrix
    distanceMatrix = []
    for i in range(len(positions)):
        dm = []
        for j in range(len(positions)):
            if i == j:
                dm.append(0)
            else:
                values = getWDTC(GFZ, positions[i], positions[j], flightAltitude, hSpeed, vSpeedMax, useCost, False)
                dm.append(values[1])
        distanceMatrix.append(dm)

    return distanceMatrix


def getTimeMatrix(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost):
    # Generate time matrix
    timeMatrix = []
    for i in range(len(positions)):
        tm = []
        for j in range(len(positions)):
            if i == j:
                tm.append(0)
            else:
                values = getWDTC(GFZ, positions[i], positions[j], flightAltitude, hSpeed, vSpeedMax, useCost, False)
                tm.append(values[1])
        timeMatrix.append(tm)

    return timeMatrix