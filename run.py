from scipy.spatial import distance_matrix

from handleGeo.coordinates.WGS84 import distance
from wdtcUAV import *


def main():

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Input values ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    GFZ = []
    positions = [[37.969951, 23.763655], [37.967441, 23.774876], [37.9689217, 23.768144], [37.9694292, 23.7731651],
                 [37.9668582, 23.7791731], [37.9684935, 23.777022], [37.9725511,23.7563531], [37.9686551,23.7654603],
                 [37.9684731,23.7763453], [37.9683001,23.7779943]] # [lat, long] in WGS84 coordinate system
    flightAltitude = 50   # m
    hSpeed = 7   # m/s
    vSpeedMax = 5   # m/s
    useCost = 50    # euro/hour
    visualize = False



    a = generateTrajectory(GFZ, positions, flightAltitude, visualize)
    print(a)



    # ~~~~~~~~~~~~~~ Example of getting Trajectories, Distance, Time and Cost values between two points ~~~~~~~~~~~~~~ #
    for i in range(len(positions)-1):
        values = getWDTC(GFZ, positions[0], positions[i+1], flightAltitude, hSpeed, vSpeedMax, useCost, visualize)
        print(f"\nTrajectory: {values[0]}\n Distance: {values[1]}\n Time: {values[2]}\n Cost: {values[3]}\n")


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~ Example of generating distance and time matrices example ~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    # If you need both
    matrices = generateMatrices(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost)
    distanceMatrix = matrices[0]
    timeMatrix = matrices[1]

    # If you need one of them
    distanceMatrix = getDistanceMatrix(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost)
    timeMatrix = getTimeMatrix(GFZ, positions, flightAltitude, hSpeed, vSpeedMax, useCost)

    for i in range(len(positions)):
        print(distanceMatrix[i])
    print()
    for i in range(len(positions)):
        print(timeMatrix[i])


    # ~ Example of providing list of Waypoints to visit and getting back overall Trajectory, Time, Distance and Cost ~ #
    # TODO: implement this function



if __name__ == "__main__":
    main()