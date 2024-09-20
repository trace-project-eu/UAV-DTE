import math
import numpy as np
from extremitypathfinder import PolygonEnvironment
from handleGeo.ConvCoords import ConvCoords
from visualization import *


def get_wdtc(gfz, initial_position, destination_position, flight_altitude, h_speed, v_speed, use_cost, visualize):
    """
    Calculate waypoints-distance-time-cost (WDTC) for UAV flight between two positions.

    Args:
        gfz (list): Geo-fenced zone and no-go-zones.
        initial_position (list): Starting position [lat, lon].
        destination_position (list): Destination position [lat, long].
        flight_altitude (float): Altitude in meters.
        h_speed (float): Horizontal speed in m/s.
        v_speed (float): Vertical speed in m/s.
        use_cost (float): Cost per hour of operation in euros.
        visualize (bool): Flag to visualize the trajectory.

    Returns:
        list: Trajectory in WGS84 coordinates, distance in meters, time in minutes, and cost in euros.
    """
    try:
        trajectories = generateTrajectory(gfz, [initial_position, destination_position], flight_altitude, visualize)
        wgs84_path, ned_path = trajectories

        # Calculate distance
        distance = calculateDistance(ned_path)

        # Estimate time
        time = estimateTime(distance, len(wgs84_path), h_speed, v_speed, 0)

        # Calculate cost
        cost = calculateCost(time, use_cost)

        return [wgs84_path, distance, time, cost]
    except Exception as e:
        print(f"Error in WDTC calculation: {e}")
        return None


def generateTrajectory(gfz, locations, flight_altitude, visualize):
    """
    Generate a UAV flight trajectory based on the GFZ and NFZs.

    Args:
        gfz (list): Geo-fenced zone and no-go-zones.
        locations (list): List of [lat, long] positions.
        flight_altitude (float): Altitude in meters.
        visualize (bool): Flag to visualize the trajectory.

    Returns:
        list: Trajectory in WGS84 coordinates and NED coordinates.
    """
    try:
        temp = np.array(locations)
        lc_alt = np.zeros((len(locations), 3))
        lc_alt[:, :-1] = temp

        # Define initial position as reference point for coordinate conversion
        convertor = ConvCoords([], [], lc_alt[0].tolist())

        # Convert wgs84 locations to NED
        ned_path = convertor.wgs84_to_ned(lc_alt.tolist())

        # Add intermediate waypoints (taking into account the flight altitude and the GFZ/NFZs)
        if not gfz:
            for i in range(0, (len(ned_path) - 1) * 3, 3):
                ned_path = np.insert(ned_path, i + 1, ned_path[i], 0)
                ned_path[i + 1][2] = flight_altitude
                ned_path = np.insert(ned_path, i + 2, ned_path[i + 2], 0)
                ned_path[i + 2][2] = flight_altitude
        else:
            environment = PolygonEnvironment()

            # Convert GFZ vertices to NED coordinates
            temp = np.array(gfz[0])
            gfz_poly = np.zeros((len(gfz[0]), 3))
            gfz_poly[:, :-1] = temp
            ned_gfz_alt = convertor.wgs84_to_ned(gfz_poly.tolist())
            polygon = np.array(ned_gfz_alt)[:, 0:2].tolist()

            list_of_holes = []
            for zone in gfz[1:]:
                temp = np.array(zone)
                nfz = np.zeros((len(zone), 3))
                nfz[:, :-1] = temp
                ned_nfz_alt = convertor.wgs84_to_ned(nfz.tolist())
                list_of_holes.append(np.array(ned_nfz_alt)[:, 0:2].tolist())

            environment.store(polygon, list_of_holes, validate=False)

            ned_path_gfz = []
            for i in range(len(ned_path) - 1):
                start_coordinates = tuple(ned_path[i][0:2])
                goal_coordinates = tuple(ned_path[i + 1][0:2])
                intermediate_path = environment.find_shortest_path(start_coordinates, goal_coordinates)[0]

                ned_path_gfz.append(ned_path[i])
                for point in intermediate_path:
                    ned_path_gfz.append(np.insert(np.array(point), 2, flight_altitude, 0))
                ned_path_gfz.append(ned_path[i + 1])

            ned_path = np.array(ned_path_gfz)

        # Convert NED path back to WGS84 coordinates
        wgs84_path = convertor.ned_to_wgs84([ned_path.tolist()])[0]

        # Visualize the path if needed
        if visualize:
            visualizeTrajectory(gfz, wgs84_path)

        return [wgs84_path, ned_path.tolist()]
    except Exception as e:
        print(f"Error in trajectory generation: {e}")
        return None


def calculateDistance(ned_path):
    """
    Calculate the total distance of a given NED path.

    Args:
        ned_path (list): Path in NED coordinates.

    Returns:
        float: Total distance in meters.
    """
    try:
        distance = sum(math.dist(ned_path[i], ned_path[i + 1]) for i in range(len(ned_path) - 1))
        return distance
    except Exception as e:
        print(f"Error calculating distance: {e}")
        return 0


def estimateTime(distance, waypoints_number, horizontal_speed, vertical_speed, delay_per_stop):
    """
    Estimate the time required for the UAV to complete a trajectory.

    Args:
        distance (float): Distance in meters.
        waypoints_number (int): Number of waypoints in the path.
        horizontal_speed (float): Horizontal speed in m/s.
        vertical_speed (float): Vertical speed in m/s.
        delay_per_stop (float): Additional delay for each stop.

    Returns:
        float: Estimated time in minutes.
    """
    try:
        a = 0.85  # Horizontal speed coefficient
        b = 0.15  # Vertical speed coefficient
        c, d = 5, 20  # Constants for calculating delay per waypoint

        balanced_speed = a * horizontal_speed + b * vertical_speed
        delay = (c * balanced_speed / (d + abs(balanced_speed))) * waypoints_number
        time = distance / (60 * balanced_speed) + delay + delay_per_stop * (waypoints_number - 1) / 3

        return time
    except Exception as e:
        print(f"Error estimating time: {e}")
        return 0


def calculateCost(time, use_cost):
    """
    Calculate the total cost of a flight based on time and cost per hour.

    Args:
        time (float): Time in minutes.
        use_cost (float): Cost per hour of operation in euros.

    Returns:
        float: Total cost in euros.
    """
    try:
        return time * use_cost / 60
    except Exception as e:
        print(f"Error calculating cost: {e}")
        return 0


def generateMatrices(gfz, positions, flight_altitude, h_speed, v_speed, use_cost):
    """
    Generate distance and time matrices for the given positions.

    Args:
        gfz (list): Geo-fenced zone and no-go-zones.
        positions (list): List of UAV positions.
        flight_altitude (float): Altitude in meters.
        h_speed (float): Horizontal speed in m/s.
        v_speed (float): Vertical speed in m/s.
        use_cost (float): Cost per hour of operation in euros.

    Returns:
        list: Distance matrix and time matrix.
    """
    try:
        distance_matrix = []
        time_matrix = []

        for i in range(len(positions)):
            dm_row = []
            tm_row = []
            for j in range(len(positions)):
                if i == j:
                    dm_row.append(0)
                    tm_row.append(0)
                else:
                    result = get_wdtc(gfz, positions[i], positions[j], flight_altitude, h_speed, v_speed, use_cost,
                                      False)
                    if result:
                        dm_row.append(result[1])  # Distance
                        tm_row.append(result[2])  # Time
                    else:
                        dm_row.append(float('inf'))
                        tm_row.append(float('inf'))
            distance_matrix.append(dm_row)
            time_matrix.append(tm_row)

        return [distance_matrix, time_matrix]
    except Exception as e:
        print(f"Error generating matrices: {e}")
        return [[], []]