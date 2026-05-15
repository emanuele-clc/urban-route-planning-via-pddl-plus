"""
encoder.py
----------
Legge i file OSM (piccola, media, grande) ed estrae i nomi delle strade
con le relative informazioni (velocità, senso unico, tipo di strada).
Salva i risultati in file .txt nella stessa cartella.
"""

import xml.etree.ElementTree as ET
import os

# Percorso base (relativo allo script)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OSM_DIR = os.path.join(BASE_DIR, "..", "osm_files")
OUT_DIR = BASE_DIR  # salva nella cartella /encoder

# File da elaborare
FILE_MAP = {
    "piccola": "dublin_piccola_centro.osm",
    "media":   "dublin_media_residenziale.osm",
    "grande":  "dublin_grande_porto.osm",
}


def estrai_strade(osm_path):
    """
    Legge un file OSM e restituisce una lista di dict con:
      - name      : nome della strada
      - highway   : tipo (secondary, residential, ...)
      - maxspeed  : velocità massima (stringa, es. "30")
      - oneway    : True/False
      - n_nodi    : numero di nodi nella way
      - way_id    : id OSM della way
    """
    tree = ET.parse(osm_path)
    root = tree.getroot()

    strade = []
    for way in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in way.findall("tag")}

        # Consideriamo solo strade percorribili in auto
        highway = tags.get("highway", "")
        if not highway or highway in ("footway", "cycleway", "path",
                                      "pedestrian", "steps", "service",
                                      "track", "construction"):
            continue

        name = tags.get("name", tags.get("ref", "(senza nome)"))
        maxspeed = tags.get("maxspeed", "?")
        oneway = tags.get("oneway", "no") == "yes"
        n_nodi = len(way.findall("nd"))
        way_id = way.get("id")

        strade.append({
            "way_id":   way_id,
            "name":     name,
            "highway":  highway,
            "maxspeed": maxspeed,
            "oneway":   oneway,
            "n_nodi":   n_nodi,
        })

    return strade


def salva_report(strade, label, out_path):
    """
    Salva il report in un file .txt leggibile.
    """
    # Ordina per nome
    strade_sorted = sorted(strade, key=lambda s: s["name"].lower())

    # Raggruppa per nome (strade con piu' tratti)
    da_nome = {}
    for s in strade_sorted:
        n = s["name"]
        if n not in da_nome:
            da_nome[n] = []
        da_nome[n].append(s)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(f"{'='*60}\n")
        f.write(f"  STRADE — zona {label.upper()}\n")
        f.write(f"  File OSM: {FILE_MAP[label]}\n")
        f.write(f"  Vie totali trovate: {len(strade)}\n")
        f.write(f"  Nomi unici: {len(da_nome)}\n")
        f.write(f"{'='*60}\n\n")

        for nome, tratti in da_nome.items():
            # Riunisci info
            speeds = set(t["maxspeed"] for t in tratti)
            highways = set(t["highway"] for t in tratti)
            oneways = all(t["oneway"] for t in tratti)
            tot_nodi = sum(t["n_nodi"] for t in tratti)

            f.write(f"  {nome}\n")
            f.write(f"    tipo      : {', '.join(sorted(highways))}\n")
            f.write(f"    velocita' : {', '.join(sorted(speeds))} km/h\n")
            f.write(f"    senso unico: {'si' if oneways else 'no (o misto)'}\n")
            f.write(f"    tratti OSM: {len(tratti)}  (nodi totali: {tot_nodi})\n")
            f.write("\n")

    print(f"  Salvato: {out_path}")


def main():
    print("=== ENCODER STRADE OSM ===\n")

    for label, filename in FILE_MAP.items():
        osm_path = os.path.join(OSM_DIR, filename)

        if not os.path.exists(osm_path):
            print(f"  [SKIP] {filename} non trovato in {OSM_DIR}")
            continue

        print(f"Elaboro '{label}' ({filename})...")
        strade = estrai_strade(osm_path)
        print(f"  Strade trovate: {len(strade)}")

        out_path = os.path.join(OUT_DIR, f"strade_{label}.txt")
        salva_report(strade, label, out_path)

    print("\nFatto! Controlla i file strade_piccola.txt, strade_media.txt, strade_grande.txt")


if __name__ == "__main__":
    main()
