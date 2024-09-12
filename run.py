from handleGeo.ConvCoords import ConvCoords
from handleGeo.coordinates import WGS84
import numpy as np

from handleGeo.coordinates.WGS84 import distance, WGS84_class


def main():

    GFZ = []
    initialPosition = [37.969951, 23.763655]       # [lat, long] in WGS84 coordinates
    destinationPosition = [37.967441, 23.774876]   # [lat, long] in WGS84 coordinates
    flightAltitude = 50   #m
    hSpeed = 5   #m/s
    vSpeed = 7   #m/s

    WGS84path = generateTrajectory(GFZ, initialPosition, destinationPosition, flightAltitude, hSpeed, vSpeed)

    # distance = calculateDistance(WGS84path)


def generateTrajectory(GFZ, initialPosition, destinationPosition, flightAltitude, hSpeed, vSpeed):

    # define initialPosition as reference point for the conversions
    convertor = ConvCoords([], [], initialPosition)

    # convert initial position and destination to NED
    nedInitialPosition = convertor.conv_wgs84_to_ned([initialPosition])[0]
    nedDestinationPosition = convertor.conv_wgs84_to_ned([destinationPosition])[0]

    # add intermediate WPs
    # TODO: add intermediate WPs for geo-fenced zones with A*
    nedPath = np.array([np.append(nedInitialPosition, 0), np.append(nedInitialPosition, flightAltitude), np.append(nedDestinationPosition, flightAltitude), np.append(nedDestinationPosition, 0)])

    # convert ned path to WGS84 coordinates
    return convertor.ned_to_wgs84([nedPath.tolist()])



def calculateDistance(WGS84path):

    # define initialPosition as reference point for the conversions
    convertor = ConvCoords([], [], WGS84path[0][0][0:2])

    # convert trajectory to NED
    nedPath = convertor.conv_wgs84_to_ned(WGS84path[0])

    distance = 0
    for i in range(len(WGS84path[0])-1):
        distance += WGS84.distance(WGS84path[0][i][0:2], WGS84path[0][i][0:2])

    return distance



def estimatedTime(WGS84path):
    #TODO: calculate time
    time = 0
    return time



if __name__ == "__main__":
    main()