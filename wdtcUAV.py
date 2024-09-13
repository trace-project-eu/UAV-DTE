import math
from handleGeo.ConvCoords import ConvCoords
import numpy as np


class wdtcUAV:
    def __init__(self, GFZ, initialPosition, destinationPosition, flightAltitude, hSpeed, vSpeed, useCost, visualize):
        self.GFZ = GFZ
        self.initialPosition = initialPosition
        self.destinationPosition = destinationPosition
        self.flightAltitude = flightAltitude
        self.hSpeed = hSpeed
        self.vSpeed = vSpeed
        self.useCost = useCost

    def getValues(self):

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        #                                              Generate Trajectory
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        # add altitude to the initial and final positions
        self.initialPosition.append(0)
        self.destinationPosition.append(0)

        # define initialPosition as reference point for the conversions
        convertor = ConvCoords([], [], self.initialPosition)

        # convert initial position and destination to NED
        nedInitialPosition = convertor.conv_wgs84_to_ned([self.initialPosition])
        nedDestinationPosition = convertor.conv_wgs84_to_ned([self.destinationPosition])

        # add intermediate WPs
        # TODO: add intermediate WPs for geo-fenced zones with A*
        nedPath = np.array([nedInitialPosition[0], np.append(nedInitialPosition[0][0:2], [self.flightAltitude]),
                                 np.append(nedDestinationPosition[0][0:2], [self.flightAltitude]),
                                 nedDestinationPosition[0]])

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
        balancedSpeed = a * self.hSpeed + b * self.vSpeed
        delay = c * balancedSpeed / (d + abs(balancedSpeed))
        time = distance / (60 * balancedSpeed) + delay
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #



        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        #                                              Calculate Cost
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        cost = time * self.useCost / 60
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #



        return [wgs84path[0], distance, time, cost]










