node_attrs = [
    {"name": "depot", "color": "black", "marker": "x"},
    {"name": "store 1", "color": "red", "marker": "o"},
    {"name": "store 2", "color": "green", "marker": "o"},
    {"name": "store 3", "color": "blue", "marker": "o"},
    {"name": "delivery", "color": "red", "marker": "x"},
    {"name": "delivery", "color": "red", "marker": "x"},
    {"name": "delivery", "color": "red", "marker": "x"},
    {"name": "delivery", "color": "green", "marker": "x"},
    {"name": "delivery", "color": "green", "marker": "x"},
    {"name": "delivery", "color": "green", "marker": "x"},
    {"name": "delivery", "color": "green", "marker": "x"},
    {"name": "delivery", "color": "blue", "marker": "x"},
    {"name": "delivery", "color": "blue", "marker": "x"},
    {"name": "delivery", "color": "blue", "marker": "x"},
]


def make_plot_data(nodes):
    node_zip = list(zip(nodes, node_attrs))
    plot_data = []
    for node in node_zip:
        coords = list(node[0])
        attrs = node[1]
        plot_data.append([coords[0], coords[1], attrs])
    return plot_data
