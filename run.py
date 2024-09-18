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
    # The user will provide the speed --> corresponding to the horizontal speed for the vehicle,
    # but we should also know the maximum vertical speed that the UAV is capable of running
    hSpeed = 7   # m/s
    vSpeedMax = 5   # m/s
    useCost = 50    # euro/hour
    visualize = False


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Define hSpeed, vSpeed ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    if hSpeed > vSpeedMax:
        vSpeed = vSpeedMax
    else:
        vSpeed = hSpeed


    # ~~~~~~~~~~~~~~ Example of getting Trajectories, Distance, Time and Cost values between two points ~~~~~~~~~~~~~~ #
    # distance in meters
    # time in minutes
    # cost in euros
    for i in range(len(positions)-1):
        values = getWDTC(GFZ, positions[0], positions[i+1], flightAltitude, hSpeed, vSpeed, useCost, visualize)
        print(f"\nTrajectory: {values[0]}\n Distance: {values[1]}\n Time: {values[2]}\n Cost: {values[3]}\n")


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~ Example of generating distance and time matrices example ~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    # If you need both
    matrices = generateMatrices(GFZ, positions, flightAltitude, hSpeed, vSpeed, useCost)
    distanceMatrix = matrices[0]
    timeMatrix = matrices[1]

    # If you need one of them
    distanceMatrix = getDistanceMatrix(GFZ, positions, flightAltitude, hSpeed, vSpeed, useCost)
    timeMatrix = getTimeMatrix(GFZ, positions, flightAltitude, hSpeed, vSpeed, useCost)

    for i in range(len(positions)):
        print(distanceMatrix[i])
    print()
    for i in range(len(positions)):
        print(timeMatrix[i])


    # ~ Example of providing list of Waypoints to visit and getting back overall Trajectory, Time, Distance and Cost ~ #
    trajectories = generateTrajectory(GFZ, positions, flightAltitude, visualize)
    wgs84Path = trajectories[0]
    trajectoryDistance = calculateDistance(trajectories[1])
    time = estimateTime(trajectoryDistance, len(wgs84Path), hSpeed, vSpeed, 2)
    cost = calculateCost(time, useCost)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Example of visualizing a trajectory with the given GFZ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    visualizeTrajectory(GFZ, np.array(trajectories[1]))


if __name__ == "__main__":
    main()