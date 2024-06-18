"""
Basic CVRP implementation representing routing strategy used at Shipt
"""

# imports
from numpy import zeros, sqrt
from numpy.random import randint, seed

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


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


# print solution
def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        total_distance += route_distance
    print(f"Total Distance of all routes: {total_distance}m")


def run_problem():
    """
    Purpose:
    """
    # Instantiate the data problem.
    data = input_data()
    # print(data["distance_matrix"])
    # print(data["pickups_deliveries"])

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc.
    def distance_callback(from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        110,  # vehicle maximum travel distance
        True,  # start cumul to zero
        "Route_Drive_Time",
    )
    time_dimension = routing.GetDimensionOrDie("Route_Drive_Time")
    time_dimension.SetGlobalSpanCostCoefficient(100)

    # Add Capacity Constraint
    def demand_callback(from_index):
        """returns demand of a given node"""
        from_node = manager.IndexToNode(from_index)

        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # no slack
        data["vehicle_capacities"],  # max capacities array
        True,  # start cumul to zero
        "Capacity",
    )

    # Define Transportation Requests.
    for request in data["pickups_deliveries"]:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index)
            <= time_dimension.CumulVar(delivery_index)
        )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    print(solution)
    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)


if __name__ == "__main__":
    run_problem()
    # dat = input_data()
    # print(dat)
    # print(len(dat["demands"]) == len(dat["nodes"]))
    # print(len(dat["distance_matrix"]) == len(dat["nodes"]))
    # print(len(dat["distance_matrix"][0]) == len(dat["nodes"]))
    # print(sum(dat["demands"]))
    # print(len(dat["pickups_deliveries"]))

    # print(dat["distance_matrix"])
    # print(dat["nodes"])

    # # shitty plot of nodes (need to figureout how to add routes)
    # import matplotlib.pyplot as plt

    # coords = dat["nodes"]

    # x, y = zip(*coords)

    # plt.scatter(x, y, color="green", marker="o")

    # for i, (x, y) in enumerate(coords):
    #     plt.text(x, y, f"{i} || {x},{y}", fontsize=9, ha="right")

    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("Where are the nodes")

    # plt.show()
