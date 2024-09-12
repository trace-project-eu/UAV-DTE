import math
from handleGeo.ConvCoords import ConvCoords
import numpy as np



class udtUAV:

    def __init__(self, GFZ, initialPosition, destinationPosition, flightAltitude, hSpeed, vSpeed, useCost, visualize):
        self.GFZ = GFZ
        self.initialPosition = initialPosition
        self.destinationPosition = destinationPosition
        self.flightAltitude = flightAltitude
        self.hSpeed = hSpeed
        self.vSpeed = vSpeed
        self.useCost = useCost

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        #                                              Generate trajectory
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
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
        self.nedPath = np.array([nedInitialPosition[0], np.append(nedInitialPosition[0][0:2], [self.flightAltitude]),
                                 np.append(nedDestinationPosition[0][0:2], [self.flightAltitude]),
                                 nedDestinationPosition[0]])

        # convert ned path to WGS84 coordinates
        self.wgs84Path = convertor.ned_to_wgs84([self.nedPath.tolist()])
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #



        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        #                                             Calculate distance
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        self.distance = 0
        for i in range(len(self.nedPath[0]) - 1):
            self.distance += math.dist(self.nedPath[i], self.nedPath[i + 1])
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #



        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        #                                              Estimate time
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
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
        self.time = self.distance / (60 * balancedSpeed) + delay
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #


        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        #                                              Calculate cost
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
        self.cost = self.time * self.useCost / 60
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #


    def getWPs(self):
        return self.wgs84Path

    def getDistance(self):
        return self.distance

    def getTime(self):
        return self.time

    def getCost(self):
        return self.cost





