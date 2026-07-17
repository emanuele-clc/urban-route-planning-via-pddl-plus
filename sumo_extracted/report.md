# Estrazione dati SUMO (semafori + turn rate)

## Zona piccola

### Semafori: 27 incroci semaforizzati (10 cluster di piu' incroci)
- Ciclo net.xml (default SUMO): tutti a 90 s
- Ciclo realistico applicato: 120 s (Dublino/SCATS)
- Ritardo semaforico medio per nodo: 7.3 s (min 0.0 / max 18.0)  [oggi in PDDL: 30 s fissi]

### Turn: 434 manovre (connection)
- Per tipo (da geometria): dritto=174, sinistra=99, inversione=81, destra=80
- Turn rate usato: 20 gradi/s
- Tempo di svolta medio (escluso 'dritto'): 5.6 s
- Coerenza direzione geometrica vs attributo SUMO 'dir': 408/434 (94%)

## Zona media

### Semafori: 97 incroci semaforizzati (45 cluster di piu' incroci)
- Ciclo net.xml (default SUMO): tutti a 90 s
- Ciclo realistico applicato: 120 s (Dublino/SCATS)
- Ritardo semaforico medio per nodo: 10.8 s (min 0.5 / max 29.4)  [oggi in PDDL: 30 s fissi]

### Turn: 4368 manovre (connection)
- Per tipo (da geometria): inversione=1406, dritto=1094, sinistra=944, destra=924
- Turn rate usato: 20 gradi/s
- Tempo di svolta medio (escluso 'dritto'): 6.3 s
- Coerenza direzione geometrica vs attributo SUMO 'dir': 4242/4368 (97%)

## Zona grande

### Semafori: 453 incroci semaforizzati (195 cluster di piu' incroci)
- Ciclo net.xml (default SUMO): tutti a 90 s
- Ciclo realistico applicato: 120 s (Dublino/SCATS)
- Ritardo semaforico medio per nodo: 8.5 s (min 0.0 / max 37.0)  [oggi in PDDL: 30 s fissi]

### Turn: 16641 manovre (connection)
- Per tipo (da geometria): inversione=4991, dritto=4591, sinistra=3580, destra=3479
- Turn rate usato: 20 gradi/s
- Tempo di svolta medio (escluso 'dritto'): 6.2 s
- Coerenza direzione geometrica vs attributo SUMO 'dir': 16022/16641 (96%)
