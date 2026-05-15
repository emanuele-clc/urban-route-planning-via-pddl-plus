import os

# Lista dei file da convertire
zone = [
    ("dublin_piccola_centro.osm", "piccola.net.xml"),
    ("dublin_media_residenziale.osm", "media.net.xml"),
    ("dublin_grande_porto.osm", "grande.net.xml")
]

for osm, net in zone:
    print(f"Sto convertendo {osm} in {net}...")
    
    # Costruiamo la stringa del comando
    comando = f"netconvert --osm-files {osm} -o {net} --geometry.remove --junctions.join --tls.guess-signals"
    
    # Eseguiamo il comando nel sistema operativo
    os.system(comando)

print("Tutte le conversioni sono finite!")