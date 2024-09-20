from wdtcUAV import *
from visualization import *

def main():

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Input values ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    # Define Geo-fenced Zones (GFZ) and No-Go-Zones (NGZs)
    # First polygon corresponds to the ROI, while the others are NGZs
    gfz = [[[37.972076, 23.758851], [37.971475, 23.760722], [37.970097, 23.763551], [37.968405, 23.765363], [37.968010, 23.766271],
            [37.967710, 23.767765], [37.967673, 23.769152], [37.967236, 23.771822], [37.966992, 23.775292], [37.965690, 23.788310],
            [37.968408, 23.789018], [37.969141, 23.782452], [37.970167, 23.781980], [37.969657, 23.776401], [37.969503, 23.775318],
            [37.968605, 23.774019], [37.969763, 23.771038], [37.969204, 23.770534], [37.970031, 23.767135], [37.971616, 23.767310],
            [37.974703, 23.760664]],
           [[37.969950, 23.765123], [37.969578, 23.766580], [37.970725, 23.766806], [37.970590, 23.765552]],
           [[37.967878, 23.774837], [37.967831, 23.775527], [37.968163, 23.775517], [37.968201, 23.774768]]]

    # List of UAV positions in WGS84 coordinate system
    positions = [[37.967701, 23.772567], [37.968005, 23.772771], [37.966957, 23.785193], [37.969143, 23.780060], [37.968105, 23.779960],
                 [37.968251, 23.777072], [37.968836, 23.767791], [37.970986, 23.763736], [37.971496, 23.766268], [37.973534, 23.760856],
                 [37.972342, 23.759800], [37.970191, 23.763856], [37.968506, 23.766270]]
    flight_altitude = 50   # in meters
    h_speed = 7   # horizontal speed in m/s
    v_speed_max = 5   # maximum vertical speed in m/s
    use_cost = 50    # euro/hour
    visualize = True

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Speed configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    v_speed = min(h_speed, v_speed_max)

    # ~~~~~~~~~~~~~~~~~~~~~~~~ Calculate Trajectory, Distance, Time, and Cost between points ~~~~~~~~~~~~~~~~~~~~~~~~~ #
    # distance in meters
    # time in minutes
    # cost in euros
    for i in range(len(positions)-1):
        values = get_wdtc(gfz, positions[0], positions[i + 1], flight_altitude, h_speed, v_speed, use_cost, visualize)
        print(f"\nTrajectory: {values[0]}\n Distance: {values[1]}\n Time: {values[2]}\n Cost: {values[3]}")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Generate Distance and Time Matrices ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    distance_matrix, time_matrix = generateMatrices(gfz, positions, flight_altitude, h_speed, v_speed, use_cost)

    print("\nDistance Matrix:")
    for row in distance_matrix:
        print(row)

    print("\nTime Matrix:")
    for row in time_matrix:
        print(row)

    # ~ Example of providing list of Waypoints to visit and getting back overall Trajectory, Time, Distance and Cost ~ #
    wgs84_path, ned_path = generateTrajectory(gfz, positions, flight_altitude, visualize)
    trajectory_distance = calculateDistance(ned_path)
    time = estimateTime(trajectory_distance, len(wgs84_path), h_speed, v_speed, 2)
    cost = calculateCost(time, use_cost)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Example of visualizing a trajectory with the given gfz ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    visualizeTrajectory(gfz, wgs84_path)

if __name__ == "__main__":
    main()