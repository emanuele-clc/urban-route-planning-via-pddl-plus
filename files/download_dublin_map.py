import osmnx as ox

# Configurazione obbligatoria per SUMO
ox.settings.all_oneway = True

# Definiamo le 3 zone con coordinate diverse
zone_configs = {
    "piccola_centro": {
        "coords": (53.3448, -6.2675), # Temple Bar / Castle
        "dist": 400,                   # Raggio molto stretto
    },
    "media_residenziale": {
        "coords": (53.3243, -6.2515), # Ranelagh / Donnybrook
        "dist": 1200,                  # Raggio medio
    },
    "grande_porto": {
        "coords": (53.3465, -6.2190), # Dublin Docklands / Port
        "dist": 3000,                  # Raggio ampio
    }
}

for nome, info in zone_configs.items():
    print(f"Scaricando {nome}...")
    
    # Scarichiamo il grafo senza semplificazione per SUMO
    G = ox.graph_from_point(info["coords"], dist=info["dist"], network_type="drive", simplify=False)
    
    # Salvataggio in formato OSM XML
    filename = f"dublin_{nome}.osm"
    ox.save_graph_xml(G, filepath=filename)
    print(f"Salvato: {filename}")

print("\nDownload completato. Ora puoi usare NETCONVERT su ogni file.")