import os

zone = [
    ("dublin_piccola_centro.osm", "piccola.net.xml"),
    ("dublin_media_residenziale.osm", "media.net.xml"),
    ("dublin_grande_porto.osm", "grande.net.xml")
]

for osm, net in zone:

    # --lefthand: Dublino (Irlanda) si guida a SINISTRA. Senza questa opzione
    # netconvert costruisce la rete per la guida a destra e in SUMO le auto
    # viaggiano dal lato sbagliato.
    comando = f"netconvert --osm-files {osm} -o {net} --geometry.remove --junctions.join --tls.guess-signals --lefthand"

    os.system(comando)

print("Tutte le conversioni sono finite!")