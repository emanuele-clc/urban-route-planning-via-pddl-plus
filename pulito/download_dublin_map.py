import osmnx as ox

# 1. Configurazione fondamentale: attiva il tracciamento dei sensi unici
ox.settings.all_oneway = True

# 2. Scarica il grafo aggiungendo simplify=False
# Questo scarica tutti i nodi necessari per il formato XML
G = ox.graph_from_place("Dublin, Ireland", network_type="drive", simplify=False)

# 3. Ora puoi salvare in XML senza errori
ox.save_graph_xml(G, filepath="dublino.osm")

print("Successo! Il file dublino.osm è pronto per NETCONVERT.")