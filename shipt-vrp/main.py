"""
Basic CVRP implementation representing routing strategy used at Shipt
"""

# imports
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

from utils import input_data, estimate_counterfactual_cost, parse_solution


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

    # Print solution on console.
    if solution:
        output = parse_solution(data, manager, routing, solution)
        performance_eval = estimate_counterfactual_cost(data, output)
        return {"route_output": output, "eval": performance_eval}


if __name__ == "__main__":
    output = run_problem()
    print(output)
    dat = input_data()
    print(dat)

    # # shitty plot of nodes (need to figureout how to add routes)
    # import matplotlib.pyplot as plt
    # from plot_utils import node_attrs

    # nodes =

    # x, y = zip(*nodes)

    # plt.scatter(x, y, color="green", marker="o")

    # for i, (x, y) in enumerate(nodes):
    #     plt.text(x, y, f"{i} || {x},{y}", fontsize=9, ha="left")

    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("Where are the nodes")

    # plt.show()
