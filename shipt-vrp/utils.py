from numpy import zeros, sqrt
from numpy.random import randint, seed


def input_data(n_vehicles=5, max_capacity=10):
    """
    Purpose:
    """

    # end def
    # randomly generate nodes x,y coords
    seed(1234)  # set seed to replicate matrix
    grid_min, grid_max = (
        0,
        100,
    )  # set x,y coordinate space (represents minutes of travel time)
    nodes = [randint(low=grid_min, high=grid_max, size=2) for x in range(14)]

    # nodes[1], nodes[2], nodes[3] represent stores w/ parcels to pick up. Remaining nodes are
    # customer delivery locations
    del_reqs = [
        [1, 4],
        [1, 5],
        [1, 6],
        [2, 7],
        [2, 8],
        [2, 9],
        [2, 10],
        [3, 11],
        [3, 12],
        [3, 13],
    ]
    demands = [0, 3, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]

    # build distance matrix
    n_nodes = len(nodes)

    dist_matrix = zeros((n_nodes, n_nodes), dtype=int)

    # Calc distances & add to matrix
    # Euclidean distance for simplicity (well pretend is estimated minutes)
    for i in range(
        1, n_nodes
    ):  # skip 0 index because all "depot" distances are 0 as routes may start any where
        for j in range(i + 1, n_nodes):
            dist = sqrt(
                (nodes[j][0] - nodes[i][0]) ** 2 + ((nodes[j][1] - nodes[i][1]) ** 2)
            )
            rounded_distance = int(round(dist))
            dist_matrix[i][j] = rounded_distance
            dist_matrix[j][i] = rounded_distance

    inputs = {
        "nodes": nodes,
        "distance_matrix": dist_matrix,
        "pickups_deliveries": del_reqs,
        "demands": demands,
        "num_vehicles": n_vehicles,
        "vehicle_capacities": [max_capacity] * n_vehicles,
        "depot": 0,  # has no effect on solution bc depot distances are all filled as 0
    }
    return inputs


def estimate_counterfactual_cost(data, route_output):
    """
    estimates a counterfactual cost and efficiency for routing solution

    counterfactual is based on expected time to fulfill all demands as single stand alone
    deliveries

    route efficiency is calculated as (routed_time - counterfactual_time) / counterfactual_time
    """
    counterfactual_time = 0
    for delivery_req in data["pickups_deliveries"]:
        arc_time = data["distance_matrix"][delivery_req[0]][delivery_req[1]]
        counterfactual_time += arc_time

    return {
        "counterfactual": counterfactual_time,
        "routed_time": route_output["total_mins"],
        "route_efficiency": (route_output["total_mins"] - counterfactual_time)
        / counterfactual_time,
    }


# print solution
def parse_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_time = 0
    routes = []
    for vehicle_id in range(data["num_vehicles"]):
        route = {"vehicle": vehicle_id, "stops": [], "route_mins": 0}
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_time = 0
        while not routing.IsEnd(index):
            stop = {}
            if manager.IndexToNode(index) != 0:
                stop["node"] = manager.IndexToNode(index)
                stop["load_activity"] = data["demands"][manager.NodeToIndex(index)]
                route["stops"].append(stop)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_time += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        route["route_mins"] = route_time
        routes.append(route)
        total_time += route_time

    # print(f"Total Minutes of all routes: {total_time}mins")
    # print(routes)
    return {"total_mins": total_time, "routes": routes}
