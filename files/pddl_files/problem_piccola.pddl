(define (problem dublin-piccola)
  (:domain dublin-navigation)

  ; 14 nodi reali del centro di Dublino (zona Temple Bar)
  ; START: Liffey Street Upper   (OSM node 659788)
  ; GOAL : Aungier Street        (OSM node 26165126)

  (:objects
    liffey_st_upper    ; Liffey Street Upper / Quay
    wellington_quay_e  ; Wellington Quay Est
    aston_quay         ; Aston Quay
    ormond_quay_w      ; Ormond Quay West
    capel_st_n         ; Capel Street Nord
    capel_st_quay      ; Capel Street / Grattan Bridge Nord
    grattan_bridge_s   ; Grattan Bridge Sud
    cork_hill          ; Cork Hill
    cork_hill_s        ; Cork Hill Sud
    dame_st_e          ; Dame Street Est
    college_green_w    ; College Green Ovest
    sgeorges_n         ; South Great George's Street Nord
    sgeorges_m         ; South Great George's Street Centro
    aungier_st         ; Aungier Street
    - location
  )

  (:init

    ; Posizione iniziale del veicolo
    (at liffey_st_upper)

    ; Contatori globali
    (= (total-dist) 0)
    (= (total-time) 0)

    ; Progress iniziale su ogni tratto = 0
    (= (progress liffey_st_upper   wellington_quay_e) 0)
    (= (progress wellington_quay_e aston_quay)        0)
    (= (progress aston_quay        ormond_quay_w)     0)
    (= (progress ormond_quay_w     capel_st_n)        0)
    (= (progress capel_st_n        capel_st_quay)     0)
    (= (progress capel_st_quay     grattan_bridge_s)  0)
    (= (progress capel_st_quay     ormond_quay_w)     0)
    (= (progress grattan_bridge_s  cork_hill)         0)
    (= (progress cork_hill         cork_hill_s)       0)
    (= (progress cork_hill_s       dame_st_e)         0)
    (= (progress cork_hill_s       grattan_bridge_s)  0)
    (= (progress dame_st_e         sgeorges_n)        0)
    (= (progress dame_st_e         cork_hill_s)       0)
    (= (progress dame_st_e         college_green_w)   0)
    (= (progress college_green_w   dame_st_e)         0)
    (= (progress sgeorges_n        dame_st_e)         0)
    (= (progress sgeorges_n        sgeorges_m)        0)
    (= (progress sgeorges_m        sgeorges_n)        0)
    (= (progress sgeorges_m        aungier_st)        0)
    (= (progress aungier_st        sgeorges_m)        0)

    ; STRADE (road ?from ?to)
    ; Solo le direzioni percorribili (rispetta senso unico OSM)
    (road liffey_st_upper   wellington_quay_e)
    (road wellington_quay_e aston_quay)
    (road aston_quay        ormond_quay_w)
    (road ormond_quay_w     capel_st_n)
    (road capel_st_n        capel_st_quay)
    (road capel_st_quay     grattan_bridge_s)
    (road capel_st_quay     ormond_quay_w)
    (road grattan_bridge_s  cork_hill)
    (road cork_hill         cork_hill_s)
    (road cork_hill_s       dame_st_e)
    (road cork_hill_s       grattan_bridge_s)
    (road dame_st_e         sgeorges_n)
    (road dame_st_e         cork_hill_s)
    (road dame_st_e         college_green_w)
    (road college_green_w   dame_st_e)
    (road sgeorges_n        dame_st_e)
    (road sgeorges_n        sgeorges_m)
    (road sgeorges_m        sgeorges_n)
    (road sgeorges_m        aungier_st)
    (road aungier_st        sgeorges_m)

    ; DISTANZE in metri (calcolate con Haversine dai dati OSM)
    (= (distance liffey_st_upper   wellington_quay_e)   82)
    (= (distance wellington_quay_e aston_quay)           9)
    (= (distance aston_quay        ormond_quay_w)       173)
    (= (distance ormond_quay_w     capel_st_n)          147)
    (= (distance capel_st_n        capel_st_quay)        46)
    (= (distance capel_st_quay     grattan_bridge_s)     83)
    (= (distance capel_st_quay     ormond_quay_w)       209)
    (= (distance grattan_bridge_s  cork_hill)           501)
    (= (distance cork_hill         cork_hill_s)          28)
    (= (distance cork_hill_s       dame_st_e)           185)
    (= (distance cork_hill_s       grattan_bridge_s)    470)
    (= (distance dame_st_e         sgeorges_n)           37)
    (= (distance dame_st_e         cork_hill_s)         185)
    (= (distance dame_st_e         college_green_w)     138)
    (= (distance college_green_w   dame_st_e)           138)
    (= (distance sgeorges_n        dame_st_e)            37)
    (= (distance sgeorges_n        sgeorges_m)          190)
    (= (distance sgeorges_m        sgeorges_n)          190)
    (= (distance sgeorges_m        aungier_st)           89)
    (= (distance aungier_st        sgeorges_m)           89)

    ; RITARDO SEMAFORICO in secondi (30 = semaforo OSM, 0 = nessun semaforo)
    ; Fonte: OSM dublin_piccola_centro.osm, highway=traffic_signals
    (= (signal-delay liffey_st_upper)   0)
    (= (signal-delay wellington_quay_e) 30)  ; 44m da semaforo OSM
    (= (signal-delay aston_quay)         0)
    (= (signal-delay ormond_quay_w)      0)
    (= (signal-delay capel_st_n)         0)
    (= (signal-delay capel_st_quay)      0)
    (= (signal-delay grattan_bridge_s)   0)
    (= (signal-delay cork_hill)         30)  ; 68m da semaforo OSM
    (= (signal-delay cork_hill_s)       30)  ; 33m da semaforo OSM
    (= (signal-delay dame_st_e)          0)
    (= (signal-delay college_green_w)    0)
    (= (signal-delay sgeorges_n)         0)
    (= (signal-delay sgeorges_m)         0)
    (= (signal-delay aungier_st)        30)  ; 14m da semaforo OSM (nodo 26165126)

    ; VELOCITA' in m/s (30 km/h = 8.33 m/s per tutte le strade)
    ; Formula: km/h * 1000 / 3600
    (= (speed liffey_st_upper   wellington_quay_e)  8.33)
    (= (speed wellington_quay_e aston_quay)         8.33)
    (= (speed aston_quay        ormond_quay_w)      8.33)
    (= (speed ormond_quay_w     capel_st_n)         8.33)
    (= (speed capel_st_n        capel_st_quay)      8.33)
    (= (speed capel_st_quay     grattan_bridge_s)   8.33)
    (= (speed capel_st_quay     ormond_quay_w)      8.33)
    (= (speed grattan_bridge_s  cork_hill)          8.33)
    (= (speed cork_hill         cork_hill_s)        8.33)
    (= (speed cork_hill_s       dame_st_e)          8.33)
    (= (speed cork_hill_s       grattan_bridge_s)   8.33)
    (= (speed dame_st_e         sgeorges_n)         8.33)
    (= (speed dame_st_e         cork_hill_s)        8.33)
    (= (speed dame_st_e         college_green_w)    8.33)
    (= (speed college_green_w   dame_st_e)          8.33)
    (= (speed sgeorges_n        dame_st_e)          8.33)
    (= (speed sgeorges_n        sgeorges_m)         8.33)
    (= (speed sgeorges_m        sgeorges_n)         8.33)
    (= (speed sgeorges_m        aungier_st)         8.33)
    (= (speed aungier_st        sgeorges_m)         8.33)

  )

  ; Obiettivo: raggiungere Aungier Street
  (:goal (at aungier_st))

  ; Minimizza il tempo totale (guida + attese ai semafori)
  (:metric minimize (total-time))

)