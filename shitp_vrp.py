"""
Basic CVRP implementation representing routing strategy used at Shipt
"""

# imports
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

from vrp_utils import input_data, estimate_counterfactual_cost, parse_solution


def run_problem():
    """
    Purpose:
    """
    # Instantiate the data problem.
    data = input_data()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    routing = pywrapcp.RoutingModel(manager)

    # Define cost (as time in minutes) of each arc.
    def drive_time_callback(from_index, to_index):
        """
        Returns the euclidian distance between two nodes. from routing matrix
        """
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(drive_time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add route length (drive time in minutes) constraint.
    # print(f'max route length {data['max_route_mins']}')
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        data["max_route_mins"],  # vehicle maximum drive time
        True,  # start cumul time at zero
        "Route_Drive_Time",
    )
    time_dimension = routing.GetDimensionOrDie("Route_Drive_Time")
    time_dimension.SetGlobalSpanCostCoefficient(100)

    # Add package Capacity Constraint
    def demand_callback(from_index):
        """returns demand (package count capacity impact) of a given node"""
        from_node = manager.IndexToNode(from_index)

        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # no slack
        data["vehicle_capacities"],  # max capacities array
        True,  # start cumul for capacity to zero
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
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
    )

    # sovle it....
    solution = routing.SolveWithParameters(search_parameters)

    # parse solution & return w/ eval data compared to counterfactual
    if solution:
        print("Solution Found")
        output = parse_solution(data, manager, routing, solution)
        performance_eval = estimate_counterfactual_cost(data, output)
        return {"route_output": output, "eval": performance_eval}


if __name__ == "__main__":
    output = run_problem()
    dat = input_data()

    for k, v in output["eval"].items():
        print(f"{k}: {v}")

    # plots of nodes / deliveries
    import matplotlib.pyplot as plt
    from numpy.random import randint

    fig, ax = plt.subplots()

    for node in dat["nodes"]:
        if node["attributes"]["name"] == "Dummy Depot":
            continue
        else:
            ax.scatter(
                node["coords"][0],
                node["coords"][1],
                color=node["attributes"]["color"],
                marker=node["attributes"]["marker"],
                label=node["attributes"]["name"],
            )

    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_title("Market Pick up / Delivery Map")
    ax.grid(True)

    # plt.legend(loc="best")
    # plt.show()
    plt.savefig("./out/nodes_plot.pdf")

    # Unpack routes and enrich with coordinates / node attributes
    routes = []
    for route in output["route_output"]["routes"]:
        stop_list = []
        if route["stops"] == []:
            continue
        else:
            for stop in route["stops"]:
                stop_detail = {
                    "x": dat["nodes"][stop["node"]]["coords"][0],
                    "y": dat["nodes"][stop["node"]]["coords"][1],
                    "color": dat["nodes"][stop["node"]]["attributes"]["color"],
                    "marker": dat["nodes"][stop["node"]]["attributes"]["marker"],
                    "name": dat["nodes"][stop["node"]]["attributes"]["name"],
                    "node_idx": stop["node"],
                    "node_activity": stop["load_activity"],
                }
                stop_list.append(stop_detail)
        routes.append(
            {
                "vehicle": route["vehicle"],
                "route_mins": route["route_mins"],
                "stops": stop_list,
            }
        )

    fig, ax = plt.subplots()

    for route in routes:
        prev_stop = None
        route_color = (
            randint(10, 100) / 100,
            randint(10, 100) / 100,
            randint(10, 100) / 100,
        )
        for stop in route["stops"]:

            ax.scatter(stop["x"], stop["y"], color=stop["color"], marker=stop["marker"])

            if prev_stop:
                ax.annotate(
                    "",
                    xy=(stop["x"], stop["y"]),
                    xytext=(prev_stop["x"], prev_stop["y"]),
                    arrowprops=dict(
                        arrowstyle="->",
                        lw=2,
                        color=route_color,
                    ),
                )
            prev_stop = stop

    # Adding plot labels and grid
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_title("Market Map w/ Routing Decisions")
    ax.grid(True)

    # Display the plot
    plt.legend()
    # plt.show()
    plt.savefig("./out/routes_plot.pdf")
