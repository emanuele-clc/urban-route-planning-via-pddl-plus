import os

# Cambia "dublin_streets.graphml" con "dublino.osm"
comando = "netconvert --osm-files dublino.osm -o dublino.net.xml --geometry.remove --ramps.guess --junctions.join --tls.guess-signals --tls.discard-loaded"

print("Inizio conversione...")
os.system(comando)
print("Conversione completata!")