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

    # plot of nodes / deliveries
    import matplotlib.pyplot as plt
    from plot_utils import make_plot_data

    plot_data = make_plot_data(dat["nodes"])

    fig, ax = plt.subplots()

    for node in plot_data:
        if node[2]["name"] == "depot":
            continue
        else:
            ax.scatter(
                node[0],
                node[1],
                color=node[2]["color"],
                marker=node[2]["marker"],
                label=node[2]["name"],
            )

    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_title("Pick up / Delivery Map")
    ax.grid(True)

    plt.legend(loc="best")
    # plt.show()
    plt.savefig("./out/nodes_plot.pdf")

    # Unpack routes and enrich with coordinates
    routes = []
    for route in output["route_output"]["routes"]:
        stop_list = []
        if route["stops"] == []:
            continue
        else:
            for stop in route["stops"]:

                stop_detail = {
                    "x": plot_data[stop["node"]][0],
                    "y": plot_data[stop["node"]][1],
                    "color": plot_data[stop["node"]][2]["color"],
                    "marker": plot_data[stop["node"]][2]["marker"],
                    "name": plot_data[stop["node"]][2]["name"],
                }
                stop_list.append(stop_detail)
        routes.append(stop_list)

    fig, ax = plt.subplots()

    for route in routes:
        prev_stop = None
        for stop in route:
            ax.scatter(stop["x"], stop["y"], color=stop["color"], marker=stop["marker"])
            if prev_stop:
                ax.annotate(
                    "",
                    xy=(stop["x"], stop["y"]),
                    xytext=(prev_stop["x"], prev_stop["y"]),
                    arrowprops=dict(arrowstyle="->", lw=1.5),
                )
            prev_stop = stop

    # Adding plot labels and grid
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_title("Scatter Plot with Individual Points and Arrows")
    ax.grid(True)

    # Display the plot
    plt.legend()
    # plt.show()
    plt.savefig("./out/routes_plot.pdf")
