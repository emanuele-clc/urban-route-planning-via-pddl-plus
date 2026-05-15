import osmnx as ox

G = ox.graph_from_place("Dublin, Ireland", network_type="drive")

ox.plot_graph(G)

ox.save_graphml(G, filepath="dublin_streets.graphml")