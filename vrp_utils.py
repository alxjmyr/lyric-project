from numpy import zeros, sqrt
from numpy.random import randint, seed
from uuid import uuid1


origins_default = [
    {"id": "Store 1", "delivery_count": 3, "color": "red"},
    {"id": "Store 2", "delivery_count": 4, "color": "green"},
    {"id": "Store 3", "delivery_count": 3, "color": "blue"},
]


def input_data(
    n_vehicles=sum([x["delivery_count"] for x in origins_default]) + 1,
    max_capacity=10,
    max_route_length=120,
    origin_locs=origins_default,
    grid_min=0,
    grid_max=100,
):
    """
    Purpose:
    n_vehicles= default to delivery count + 1 to ensure it is not a constraint
    max_capacity= number of orders a vehicle can carry at a given time
    max_route_length= max number of minutes of drive time for a route
    origin_locs = dict w/ info about origin attributes and requested delivery counts
    grid_min & grid_max set size of x,y grid for the problem space
    """
    seed(123)  # set seed to ensure location consistency across runs
    nodes = [
        {
            "coords": randint(low=grid_min, high=grid_max, size=2),
            "attributes": {"name": "Dummy Depot", "color": "black"},
        }
    ]  # start w/ dummy depot node
    delivery_requests = []
    demands = [0]  # dummy depot node w/ demand set to zero

    # for each origin generate delivery requests and associated nodes
    # will generate a node for each pair of origin and delivery in a delivery request so that multiple vehicles can
    # service a node... Down side is a roughly n^2 increase in matrix size
    for origin in origin_locs:

        # generate origin location coords
        origin_id = origin["id"]
        origin_coords = randint(
            low=grid_min, high=grid_max, size=2
        )  # create x,y coords for origin location

        n_deliveries = origin["delivery_count"]

        # generate each delivery location and add the origin & destination to node list
        for delivery in range(n_deliveries):
            nodes.append(
                {
                    "coords": origin_coords,
                    "attributes": {
                        "name": origin_id,
                        "color": origin["color"],
                        "marker": "o",
                    },
                }
            )
            origin_idx = (
                len(nodes) - 1
            )  # keep track to index of current origin for building demands & delivery req's arrays
            demands.append(1)  # add demand for origin node

            delivery_id = uuid1()
            delivery_coords = randint(low=grid_min, high=grid_max, size=2)
            nodes.append(
                {
                    "coords": delivery_coords,
                    "attributes": {
                        "name": delivery_id,
                        "color": origin["color"],
                        "marker": "x",
                    },
                }
            )  # add delivery location to nodes
            delivery_idx = len(nodes) - 1
            demands.append(-1)  # add demand sink for delivery node

            # create delivery request (origin_idx, delivery_idx)
            delivery_requests.append([origin_idx, delivery_idx])

    # build time matrix for vrp problem
    n_nodes = len(nodes)

    dist_matrix = zeros((n_nodes, n_nodes), dtype=int)

    # Calc distances & add to matrix
    # Euclidean distance for simplicity (can predent that its estimated fulfillment time from a fancy ML model)
    for i in range(
        1, n_nodes
    ):  # skip 0 index because all "depot" distances are 0 as routes may start any where
        for j in range(i + 1, n_nodes):
            dist = sqrt(
                (nodes[j]["coords"][0] - nodes[i]["coords"][0]) ** 2
                + ((nodes[j]["coords"][1] - nodes[i]["coords"][1]) ** 2)
            )
            rounded_distance = int(round(dist))

            # insert rounded distance & create symmetrical
            # in the real world w/ ML predictions that may not be the case
            # A >> B and B >> A are not necessarily identical trips
            dist_matrix[i][j] = rounded_distance
            dist_matrix[j][i] = rounded_distance

    inputs = {
        "nodes": nodes,
        "distance_matrix": dist_matrix,
        "pickups_deliveries": delivery_requests,
        "demands": demands,
        "num_vehicles": n_vehicles,
        "vehicle_capacities": [max_capacity] * n_vehicles,
        "max_route_mins": max_route_length,
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
        "route_efficiency": round(
            (route_output["total_mins"] - counterfactual_time) / counterfactual_time, 3
        ),
    }


# print solution
def parse_solution(data, manager, routing, solution):
    """Creates a log of created routes"""
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
