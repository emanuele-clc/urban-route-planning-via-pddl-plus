# Comandi — Dublin PDDL+ Navigator

## Avviare la webapp

```bash
cd files/webapp
python app.py
```
Poi apri il browser su **http://localhost:5000**

---

## Visualizzare il percorso in SUMO

### Rotte predefinite (piccola / media / grande)
```bash
cd files
python sumo_visualize.py piccola
python sumo_visualize.py media
python sumo_visualize.py grande
```

### Rotta personalizzata (dopo aver usato la webapp)
```bash
cd files
python sumo_visualize.py pddl pddl_files/problem_custom.pddl piccola
```
> Sostituisci `piccola` con `media` o `grande` a seconda della zona usata nella webapp.

Una volta aperto sumo-gui:
- **▶ Play** per avviare la simulazione
- **Ctrl+A** per adattare la vista
- Click destro sull'auto → **Track** per seguirla

---

## Risolvere un problema PDDL+ con ENHSP (da terminale)

```bash
cd files/pddl_files
python run.py piccola
python run.py media
python run.py grande
```

---

## Scaricare le mappe OSM

```bash
cd files
python download_dublin_map.py
```

---

## Rigenerare i file PDDL dalle mappe OSM

```bash
cd files
python build_problems.py
```

---

## Installare le dipendenze (prima volta)

```bash
pip install flask osmnx up-enhsp
```

> Serve anche **Java 17+** per ENHSP e **SUMO** per la visualizzazione.
