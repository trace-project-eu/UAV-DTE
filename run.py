from wdtcUAV import *
from visualization import *

def main():

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Input values ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    # First polygon corresponds to the ROI and all the others to NGZs
    GFZ = [[[37.972076, 23.758851], [37.971475, 23.760722], [37.970097, 23.763551], [37.968405, 23.765363], [37.968010, 23.766271],
            [37.967710, 23.767765], [37.967673, 23.769152], [37.967236, 23.771822], [37.966992, 23.775292], [37.965690, 23.788310],
            [37.968408, 23.789018], [37.969141, 23.782452], [37.970167, 23.781980], [37.969657, 23.776401], [37.969503, 23.775318],
            [37.968605, 23.774019], [37.969763, 23.771038], [37.969204, 23.770534], [37.970031, 23.767135], [37.971616, 23.767310],
            [37.974703, 23.760664]],
           [[37.969950, 23.765123], [37.969578, 23.766580], [37.970725, 23.766806], [37.970590, 23.765552]],
           [[37.967878, 23.774837], [37.967831, 23.775527], [37.968163, 23.775517], [37.968201, 23.774768]]]
    positions = [[37.967701, 23.772567], [37.968005, 23.772771], [37.966957, 23.785193], [37.969143, 23.780060], [37.968105, 23.779960],
                 [37.968251, 23.777072], [37.968836, 23.767791], [37.970986, 23.763736], [37.971496, 23.766268], [37.973534, 23.760856],
                 [37.972342, 23.759800], [37.970191, 23.763856], [37.968506, 23.766270]] # [lat, long] in WGS84 coordinate system
    flightAltitude = 50   # m
    # The user will provide the speed --> corresponding to the horizontal speed for the vehicle,
    # but we should also know the maximum vertical speed that the UAV is capable of running
    hSpeed = 7   # m/s
    vSpeedMax = 5   # m/s
    useCost = 50    # euro/hour
    visualize = True



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
    visualizeTrajectory(GFZ, trajectories[0])


if __name__ == "__main__":
    main()