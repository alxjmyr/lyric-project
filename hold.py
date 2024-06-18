"""
Basic CVRP implementation representing routing strategy used at Shipt
"""

# imports
from numpy import zeros, sqrt
from numpy.random import randint

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


def input_data():
    """
    Purpose:
    """

    # end def
    # randomly generate nodes for all pick up and deliveries
    # node 1 will be depot 1, node 10 will be depot 2
    # node 2-9 are deliveries from depot 1, nodes 11-21 are deliveries from depot 2
    grid_min, grid_max = 0, 200  # set x,y coordinate space
    nodes = [randint(low=grid_min, high=grid_max, size=2) for x in range(21)]

    depot_1_deliveries = [[1, x + 1] for x in range(1, 9)]
    depot_2_deliveries = [[10, x + 1] for x in range(10, 21)]

    # build distance matrix
    n_nodes = len(nodes)

    dist_matrix = zeros((n_nodes, n_nodes), dtype=int)

    # Calc distances & add to matrix
    # Euclidean distance for simplicity
    for i in range(n_nodes):
        for j in range(i + 1, n_nodes):
            dist = sqrt(
                (nodes[j][0] - nodes[i][0]) ** 2 + +((nodes[j][1] - nodes[i][1]) ** 2)
            )
            rounded_distance = int(round(dist))
            dist_matrix[i][j] = rounded_distance
            dist_matrix[j][i] = rounded_distance

    inputs = {
        "distance_matrix": dist_matrix,
        "pickups_deliveries": depot_1_deliveries + depot_2_deliveries,
        "num_vehicles": 6,
        "depot": 0,  # modify later to reflect arbitrary start / end as an option
    }
    return inputs


# print solution
def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    for vehicle_id in range(data["n_vehicles"]):
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
    print(data["distance_matrix"])
    print(data["pickups_deliveries"])

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
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Define Transportation Requests.
    for request in data["pickups_deliveries"]:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index)
            <= distance_dimension.CumulVar(delivery_index)
        )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)


if __name__ == "__main__":
    run_problem()
