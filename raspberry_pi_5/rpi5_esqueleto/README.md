# Software de la Raspberry Pi 5 (IEE2913, Grupo 9) — lazo externo de seguimiento

Programa único que corre en la RPi. Hace lo siguiente:
- detecta el láser rojo;
- ubica el objetivo en el mundo con la pose real de la plataforma (AS5600, por telemetría);
- predice dónde estará el objetivo cuando el ESP32 aplique la consigna;
- le manda θ_ref (y ω_ff, opcional) al firmware `esp_32/esp32_pan_tilt`, que cierra el lazo de posición con el PI de `pid_t_2.pdf`.

El mismo código corre en el PC con una cámara simulada y un ESP32 simulado, en **lazo cerrado**.

## Estructura y trazabilidad a los diagramas de bajo nivel

| Archivo | Bloque | Qué hace |
|---|---|---|
| `pt/parametros.py` | PI-7 y todos | Todos los números con unidades. Ya actualizado a las decisiones del 25 % (80 fps, exposición 2 ms, RLS orden 3 con λ = 0,80) |
| `pt/camara.py` | PI-1 | `FuenteCamara`: planos Y y V, balance de blancos fijo. `FuenteSintetica`: láser rojo, lámpara blanca opcional y escena que responde a la pose simulada |
| `pt/deteccion.py` | PI-2 a PI-6 | `DetectorForma` (avión y objeto circular por FORMA, plano Y), `DetectorLaserRojo` (plano V con confirmación en Y, traído de `laser_op.py`), `DetectorLuz`, `DetectorLaser` (brillo, el original) y `DetectorAvionRojo` (avión por color, respaldo) |
| `pt/formas.py` | PI-3 / PI-6 | Descriptores de Fourier del contorno, normalización afín y banco de plantillas (incorporadas y aprendidas en `formas.json`) |
| `pt/geometria.py` | CS-1 | Píxel → ángulo, y **dirección del objetivo en el mundo** con la pose actual (composición exacta de rotaciones) |
| `pt/prediccion.py` | EP-1 a EP-5 | Gating conjunto de pan y tilt, reinicio informado (sin pasar por PÉRDIDA), RLS del equipo (`pt/rls_predictor.py`, sin cambios), predicción y velocidad al instante de aplicación |
| `pt/control.py` | CS-2 | `ReferenciaVisual`: θ_ref y ω_ff a partir de la estimación; retención fuera de SEGUIMIENTO |
| `pt/enlace.py` | CS-3 | `EnlaceTrama` (ESP32 real, telemetría en un hilo), `EnlaceSimulado` (planta de pid_t_2 y el mismo PI) y `EnlaceNulo` |
| `pt/modelo_eje.py` | Modelo | Planta de pid_t_2, réplica exacta del PI del firmware, polos, Routh, métricas y ajuste de ζ |
| `pt/supervisor.py` | SE-2 / SE-3 | BÚSQUEDA ⇄ SEGUIMIENTO ⇄ PÉRDIDA; MODO_SEGURO por enlace caído o falla del ESP32; HOMING por AS5600 |
| `pt/registro.py` | MR-2 / CTL-7 | CSV por cuadro: pose, objetivo, consigna, telemetría y error de centrado |
| `pt/monitor_red.py` | MON-1 / MON-2 | Servidor web del tablero en un hilo aparte: flujo en vivo, exportación CSV y `POST /ajustes` (ajustes en vivo) |
| `pt/recorte.py` | PI-2 | Recorte de la zona de trabajo (cuadrilátero elegido en el tablero): lo de afuera no lo ve el detector. Opcional y apagado por defecto (sección 6) |
| `pt/ajustes.py` | MON-1 | Parámetros que el tablero puede cambiar en caliente: validación de tipo y rango, cola hacia el lazo y perfiles por objetivo |
| `pt/monitor_web/index.html` | MON-1 | Tablero: imagen, detección, objeto, error, ángulos, referencia, ω_cmd, límites, fallas y modo |
| `main.py` | — | Une todo. `Ventana` (HDMI) y `MonitorRed` (red local) |
| `herramientas/respuesta_escalon.py` | CTL-7 | Escalón real contra el modelo de pid_t_2; estima ζ (es decir, b) |
| `herramientas/analizar_registro.py` | CTL-7 | Presupuesto medido de una corrida (latencias, detección, error de centrado, atraso) y comparación por tramos |
| `herramientas/comparar_ganancias_ff.py` | CTL-7 | Tabla y figuras: con y sin feedforward para cada grupo de ganancias del PI, simulado y medido |
| `herramientas/fijar_ganancias.py` | CTL-7 | Fija en el ESP32 las ganancias de un grupo (comando `k` de la consola), con `main.py` detenido |
| `herramientas/aprender_forma.py` | DET-2 | Aprende una vista del objetivo real (cámara o fotos) y la guarda en `formas.json` |
| `herramientas/grabar_cuadros.py` | DET-2 | Graba cuadros (plano Y, PNG sin pérdida) con la cámara, sin la torreta, para probar el detector después en el PC |
| `herramientas/reproducir_forma.py` | DET-6 | Corre el detector por forma sobre cuadros grabados: tabla por etiqueta, CSV, imágenes anotadas, `--set` y `--barrer` de parámetros |
| `herramientas/evaluar_forma.py` | DET-6 | Aciertos, falsos positivos, error de centroide y tiempo del detector por forma, en escenas sintéticas |
| `pruebas_unitarias/` | — | 49 pruebas sin hardware (43 del esqueleto y 6 del recorte) |

Los comentarios usan colores: `#!` rojo (seguridad y riesgos), `#*` verde (lo importante), `#?` azul (supuestos por verificar), `#todo` naranjo (pendientes).

## Cómo se reparte el lazo

```
RPi (80 fps)                                                 ESP32 (1 kHz, pid_t_2)
cámara -> (u,w) -> objetivo en el mundo = f(u, w, θ_med(t_exp)) ─┐
                    -> RLS -> θ_ref = objetivo(t_aplicación) ─────┼──► e = θ_ref − θ_med -> PI -> ω_cmd -> STEP
                    -> ω_ff = velocidad del objetivo (opcional) ──┘                  ▲
                                         telemetría θ_med (200 Hz) ◄─────────────────┘
```

Se usa θ_med en el **instante de exposición** porque entre la foto y la llegada de la consigna (unos 18 ms) la plataforma ya se movió. Si se sumara Δθ a la posición del momento, ese movimiento se contaría dos veces.

## 1. En el PC

```bash
pip install numpy pyserial matplotlib
python pruebas_unitarias/test_esqueleto.py                 # 43 pruebas
python pruebas_unitarias/test_recorte.py                   # 6 pruebas del recorte
python main.py                                             # visión sola, cámara fija
python main.py --enlace simulado --lampara                 # lazo cerrado simulado, PI puro
python main.py --enlace simulado --lampara --feedforward   # ídem con ω_ff
python herramientas/respuesta_escalon.py --solo-modelo --eje pan --kp 5 --ki 6.25 --escalon 10
```

## 2. En la Raspberry Pi

**Enlace con el ESP32 por el UART del header** (cableado del 22-10): RX0 del ESP32 al pin 8 (GPIO14, TXD), TX0 al pin 10 (GPIO15, RXD) y GND común. El puerto es `/dev/ttyAMA0`, que ya es el valor por defecto de `--puerto`. Hay que habilitarlo una sola vez:

```bash
echo "dtparam=uart0=on" | sudo tee -a /boot/firmware/config.txt
sudo reboot
ls -l /dev/ttyAMA0          # debe existir
```

No usar `/dev/serial0`: en la RPi 5 apunta al conector de depuración (`ttyAMA10`), no a los pines 8 y 10.

**Detectores:**
- `--detector rojo` es el láser rojo, por color.
- `--detector luz` es el flash del celular. Viene de `mi_proyecto_camara/luz_2.py`. Ninguna otra luz blanca intensa puede quedar en el cuadro.
- `--detector brillo` es el original.
- `--detector avion` es el avión **por forma** (silueta en el plano Y, sin usar el color); `ovni` es lo mismo para un objeto circular; `avion_rojo` es el detector por color anterior, que queda de respaldo. Ver la sección 5.

Antes de medir: `vcgencmd get_throttled` debe dar `0x0`. Dependencias: `sudo apt install -y python3-serial python3-opencv python3-matplotlib`.

```bash
python3 pruebas_unitarias/test_esqueleto.py
python3 main.py --fuente camara --duracion 20 --ventana            # 1. láser real, sin motores: revisar detección
python3 main.py --fuente camara --enlace trama --duracion 20   # 2. con ESP32, motores deshabilitados
python3 herramientas/respuesta_escalon.py --puerto /dev/ttyAMA0 --eje pan --escalon 3 --habilitar   # 3. modelo
python3 main.py --fuente camara --enlace trama --duracion 30 --habilitar      # 4. LAZO CERRADO
```

- En el paso 1, si el láser no se detecta, revisar `pico` y `motivo` en el CSV. Si el pico de V queda bajo 170, subir `ganancias_color[0]` (rojo) o bajar `exposicion_us`.
- El paso 2 sirve para comprobar que la telemetría llega y que `pan_med`/`tilt_med` se mueven al girar los ejes a mano. **Sin `--habilitar` la RPi nunca mueve los motores:** manda "deshabilitar" en vez de consignas, aunque el ESP32 haya quedado energizado de una corrida anterior.
- La pose se lee en el instante de la **fila** donde está el objetivo (persiana rodante del IMX219; `t_lectura_ms` en `parametros.py` es una estimación por medir).
- El procedimiento de signos y cero del ESP32 está en `esp_32/esp32_pan_tilt/README.md`, y se hace **antes** del paso 3.

## 3. Monitoreo por la red, sin monitor HDMI (MON-1 / MON-2)

`--monitor` levanta un tablero web en la propia RPi, en hilos aparte del lazo. Se abre desde el navegador del PC o del celular, conectados a la misma red que la RPi.

```bash
# [RPi]
python3 main.py --fuente camara --enlace trama --duracion 0 --monitor
# [PC] navegador:  http://g9-cap.local:8000   (o http://<IP de la RPi>:8000)
```

- **Muestra (MON-1):**
  - estado de detección;
  - posición del objeto en la imagen y en el mundo;
  - error de centrado en pan y tilt;
  - θ_med por eje y referencia de centrado (centro de la imagen y θ_ref);
  - ω_cmd por actuador;
  - límites y finales de carrera;
  - las 8 fallas del ESP32, el enlace y el modo seguro;
  - modo de operación (supervisor y ESP32);
  - la imagen con el objeto y la consigna superpuestos.
- **Dinámica (MON-2):** cada refresco (20 por segundo) trae **todos** los cuadros nuevos, así que los gráficos tienen los 80 Hz completos. Tienen cursor sincronizado, ventana de 5 a 60 s y pausa.
- **Exportar (MON-2):**
  - `/historial.csv`: últimos 120 s, una fila por cuadro;
  - `/telemetria.csv`: θ_med, θ_ref y ω_cmd del ESP32 a 200 Hz;
  - `/registro.csv`: el CSV completo de la corrida.
- **Ajustes en vivo (desde el 17-11-2026):** el tablero ya no es de sólo lectura. Ver la sección 4.
- Al terminar la corrida, el tablero queda abierto para exportar; se cierra con `Ctrl+C`.
- **Sin dependencias nuevas:** usa la biblioteca estándar, y OpenCV para la imagen. Opciones en `python3 main.py --help`, grupo *monitoreo por red*.
- **Prueba en el PC:** `python main.py --enlace simulado --lampara --monitor --duracion 30`, y abrir `http://localhost:8000`.

## 4. Ajustes en vivo: un solo `main.py` para todos los objetivos

La tarjeta **Ajustes en vivo** del tablero cambia parámetros con el lazo corriendo. Ya no hay que volver a ejecutar `main.py` para pasar del láser a la luz o al avión.

```bash
# [RPi] una sola ejecución para toda la demostración
python3 main.py --fuente camara --enlace trama --habilitar --duracion 0 --monitor
# [PC o celular] http://g9-cap.local:8000  ->  tarjeta "Ajustes en vivo"
```

| Grupo | Qué se puede cambiar | Efecto en el lazo |
|---|---|---|
| Cámara | exposición, ganancia analógica, ganancias de color (rojo, azul) | `set_controls` de Picamera2; la cámara lo toma unos cuadros después. El tablero muestra lo que la cámara informa |
| Objetivo | detector: `rojo`, `luz`, `avion`, `ovni`, `avion_rojo`, `brillo` | detector y estimador parten de cero y el supervisor vuelve a BÚSQUEDA; la plataforma **retiene** hasta adquirir el objetivo nuevo |
| Detector | umbrales del detector activo (en `avion` y `ovni`: polaridad, contraste, parecido exigido, ancho máximo, área mínima, alargamiento, borde, quitar lo delgado, rechazo de saltos) | inmediato, sin interrumpir el seguimiento |
| Recorte de imagen | zona de trabajo (cuadrilátero) para todos los detectores | inmediato; apagado por defecto (sección 6) |
| Predicción | λ (0,60 a 0,99), piso del gating, orden del RLS, feedforward ω_ff | λ, gating y ω_ff sin interrupción; cambiar el orden reinicia el estimador |

- **Perfiles por objetivo** (`laser`, `luz`, `avion`): un clic aplica detector, cámara, umbrales y λ juntos.
  - Los valores iniciales están en `PERFILES` de `pt/parametros.py`. Son valores de partida, no medidos en la plataforma.
  - **Guardar perfil** escribe los valores vigentes en `perfiles.json` (junto a `main.py`), que tiene prioridad en las corridas siguientes. Conviene subirlo al repositorio una vez ajustado.
- **Cómo viaja un cambio:** el navegador hace `POST /ajustes`; el servidor valida clave, tipo y rango (`pt/ajustes.py`) y lo encola; el lazo lo aplica al comienzo del cuadro siguiente y lo confirma. El tablero muestra siempre el valor vigente.
- **Registro:** el CSV tiene las columnas `detector`, `exp_us`, `ganancia`, `lam` y `ff` (feedforward activo), así que una corrida con varios objetivos se puede separar después.
- **Análisis:** `herramientas/analizar_registro.py` parte cada CSV en tramos con el mismo detector y feedforward, analiza cada uno y entrega una tabla comparativa.

**Comparar con y sin feedforward, eligiendo el objetivo desde el tablero:**

```bash
# 1. sin feedforward: ejecutar, elegir el perfil (p. ej. "avion") en el tablero, mover el objetivo, Ctrl+C
python3 main.py --fuente camara --enlace trama --habilitar --monitor --duracion 0 --registro registros/sin_ff
# 2. con feedforward: igual, repitiendo el mismo movimiento
python3 main.py --fuente camara --enlace trama --habilitar --monitor --duracion 0 --feedforward --registro registros/con_ff
# 3. comparación: sólo los tramos con el detector elegido
python3 herramientas/analizar_registro.py --detector avion registros/sin_ff/*.csv registros/con_ff/*.csv
```

También sirve una sola corrida, marcando la casilla de feedforward a mitad de camino: el paso 3 es el mismo, con ese único CSV.

**Comparar grupos de ganancias, con y sin feedforward** (`herramientas/comparar_ganancias_ff.py`). Grupo A: arranque de `config.h` (pan 3 / 2,25; tilt 10 / 25). Grupo B: `pid_t_2.pdf` §8 (pan 5 / 6,25; tilt 20 / 100).

```bash
# Simulación (PC, ~1,5 min): cuatro corridas de main.py --enlace simulado --ganancias-sim ...
python3 herramientas/comparar_ganancias_ff.py
# Después del laboratorio: agrega las filas "medido" (una carpeta de registros por grupo)
python3 herramientas/comparar_ganancias_ff.py --reusar --detector avion --lab-a registros/lab_A/*.csv --lab-b registros/lab_B/*.csv
```

Deja en `docs/figuras/` la tabla (`comp_ganancias_ff.csv` y `.md`) y las figuras (`comp_ganancias_ff_barras`, `comp_ganancias_ff_traza_sim` y `_traza_lab`, en PNG y PDF). En el ESP32 real las ganancias se cambian con `python3 herramientas/fijar_ganancias.py B` (manda `k pan 5 6.25` y `k tilt 20 100` por el UART del enlace, con `main.py` detenido; no quedan guardadas tras un reinicio). `--ganancias-sim` sólo afecta al lazo simulado.
- **Detección por forma (revisión del 18-11):** el detector `avion` ya no usa el color: reconoce la silueta en el plano Y (contraste local y descriptores de Fourier del contorno). `ovni` busca una figura circular con el mismo método. El detector por color anterior queda como `avion_rojo`. Requieren OpenCV. Antes de usarlo con el avión real hay que aprender sus vistas: `python3 herramientas/aprender_forma.py --nombre avion_arriba --camara --exposicion 6000 --ganancia 2`. Prueba en el PC: `python3 main.py --enlace simulado --objeto avion --detector avion --feedforward`. Método, resultados y condiciones de operación: `docs/deteccion_por_forma.md`. Flujo de trabajo completo: sección 5.
- **Predictor (revisión del 17-11):** el tablero tiene tres controles más en el grupo Predicción: "Reinicio sin pasar por PÉRDIDA", "Gating conjunto (pan y tilt)" y "Oclusión: tope de la aceleración" (apagado, valor 0). Apagar los dos primeros deja el predictor como antes. El CSV agrega `gating_d` (innovación en veces el umbral; mayor que 1 es rechazo), `sigma_pan` y `sigma_tilt`, y `est_u` / `est_w` pueden valer `reinicio`. `analizar_registro.py` los resume en su punto 5, para ajustar el piso del gating con datos. Detalle y mediciones: `docs/prediccion_rls.pdf`, sección 6.1.
- **Riesgo asumido:** cualquier equipo conectado a la misma red puede cambiar estos valores, y con `--habilitar` un cambio de objetivo o de umbral termina moviendo la plataforma.
  - No existe ruta para habilitar motores, mandar referencias o comandos al ESP32, ni para cambiar límites o ganancias del PI.
  - `--monitor-solo-lectura` deja el tablero como antes.
  - Si el objetivo nuevo no está en cuadro, la plataforma se queda quieta: el patrón de búsqueda sigue pendiente.
- **Prueba en el PC:** `python main.py --enlace simulado --lampara --monitor --duracion 60`. Con fuente sintética no aparece el grupo Cámara.

## 5. Detección del avión por forma: flujo de trabajo (revisión del 18-11)

**Por qué:** las pruebas serán a contraluz, así que el color y el brillo no sirven. `avion` reconoce la silueta: contraste local en el plano Y, contorno, descriptor de Fourier y comparación con las plantillas de `formas.json`. Método y resultados: `docs/deteccion_por_forma.md`.

**Plantillas.** Las genéricas (`avion_planta`, `avion_perfil`) no calzan con el avión real. Hay que aprender cada vista con el avión del equipo:

```bash
python3 herramientas/aprender_forma.py --nombre avion_planta --camara --exposicion 2000 --ganancia 1
python3 herramientas/aprender_forma.py --nombre avion_lejos  --imagen registros/capturas/avion_lado_2m/*.png   # o desde cuadros grabados
python3 herramientas/aprender_forma.py --listar
```

- **El nombre debe empezar con `avion_`.** Con otro nombre el detector no la usa y sigue con las genéricas.
- Cuando hay vistas aprendidas, mandan ellas. Una plantilla por cara del avión, y se puede aprender una a cada distancia.
- El avión se debe ver completo, sobre fondo liso y con un soporte delgado (hilo o palo de menos de unos 8 mm a 2 m). Un palo grueso se funde con la silueta.

**Probar sin instalar la torreta.** Basta la cámara de la RPi apuntando a la escena:

```bash
# [RPi, con main.py detenido: la cámara no se comparte]
python3 herramientas/grabar_cuadros.py --etiqueta avion_planta_2m --distancia 2 --exposicion 2000 --ganancia 1
python3 herramientas/grabar_cuadros.py --etiqueta sin_avion --cuadros 30 --exposicion 2000 --ganancia 1   # "sin_..." = escena sin avión
# [PC] copiar registros/capturas con scp -r y probar cuantas veces haga falta
python3 herramientas/reproducir_forma.py registros/capturas/* --roi "x1,y1,x2,y2,x3,y3,x4,y4"
python3 herramientas/reproducir_forma.py registros/capturas/* --set dist_max=0.25 --set contraste_min=60
python3 herramientas/reproducir_forma.py registros/capturas/* --barrer dist_max=0.15,0.2,0.25,0.3
python3 herramientas/reproducir_forma.py registros/capturas/avion_lado_2m --secuencia   # cuadros seguidos (ventana y salto)
```

- `reproducir_forma.py` entrega por etiqueta el % de cuadros válidos, la distancia mediana, el tiempo y los motivos de rechazo. Una detección válida en una etiqueta `sin_...` cuenta como falso positivo. Los PNG anotados quedan en `registros/reproduccion/`.
- `--roi` toma el texto que muestra el tablero al activar el recorte, y vale también para `aprender_forma.py`.
- Cada carpeta de `registros/capturas/` lleva `meta.json` (exposición, ganancia, distancia, nota) y `contacto.jpg` (miniaturas para revisar a ojo).

**Ajustes del tablero que más pesan** (grupo Detector, con `avion` u `ovni` seleccionado):

| Control | Para qué |
|---|---|
| Contraste mínimo | A contraluz el avión es casi negro: subirlo (60 a 80) deja fuera hojas y sombras tenues |
| Parecido exigido (`dist_max`) | Con plantilla aprendida, 0,15 a 0,25. Más alto deja pasar figuras ajenas (hojas, marcos) |
| Alargamiento máximo | De lejos las alas se pierden y el avión queda largo y angosto: si el motivo es «figuras muy alargadas», subirlo |
| Dónde se toma el borde | Bajarlo (0,30 a 0,35) conserva las alas finas o desenfocadas |
| Quitar lo delgado (palo) | Borra lo más delgado que una fracción de √área. Muy alto se come las alas; 0 no borra nada |
| Rechazar saltos entre cuadros | Desmarcarlo para ajustar sin esa protección; el gating del estimador sigue activo |

**Condiciones de la prueba (ventana de fondo):**
- Exposición baja (1000 a 3000 µs, ganancia 1): ventana clara, avión negro. Con exposición alta la luz se come el borde y el avión se ve más chico.
- A 2 m el avión mide unos 190 a 280 px de área, bajo los 400 px que recomienda `aprender_forma.py`. Es el límite del método y el margen es poco.
- Una vista de canto (más de 6 a 1 de largo contra ancho) se descarta. Mantener el avión de planta, o aprender también el lado.
- Si el avión pasa delante de hojas oscuras, su silueta se junta con ellas y ningún detector por forma las separa. Dejar el avión sobre la parte despejada de la ventana y poner el recorte sobre ella.

## 6. Recorte de la zona de trabajo (revisión del 18-11)

Quita de la vista del **detector** todo lo que está fuera de un cuadrilátero (la pared, los objetos de alrededor, el marco de la ventana). Vale para todos los detectores y parte **apagado**.

- **Uso:** en el tablero, «Dibujar recorte» y 4 clics sobre la imagen (el orden no importa), o la casilla «Recorte activo». «Quitar recorte» lo apaga. La zona excluida se ve oscurecida en la imagen del tablero, que sigue mostrando el cuadro completo. Con el recorte activo, el tablero muestra el texto `--roi "..."` para las herramientas.
- **Qué cambia y qué no:** sólo la imagen que recibe `det.procesar()`. Lo de afuera se rellena con la mediana de lo de adentro, para no crear un borde falso. No se recorta ni se reescala el cuadro, así que `(u, w)`, la geometría (0,115 °/px), la pose, el control y el registro siguen iguales. Cuesta unos 0,2 a 0,3 ms por cuadro.
- **No va en los perfiles:** depende de dónde esté la cámara, no del objetivo.
- **Un cuadrilátero inválido no se activa:** puntos alineados, cóncavo o de menos del 1 % del cuadro (el tablero vuelve a mostrar «apagado»).
- **Cómo volver al comportamiento anterior:**
  - desmarcar «Recorte activo» (el detector recibe el cuadro tal cual, sin copiarlo);
  - arrancar con `--sin-recorte` (no aparecen controles ni código en el lazo);
  - si algo falla dentro del recorte, se apaga solo y el detector sigue con el cuadro completo.
- **Verificado en simulación:** con el recorte apagado, la detección es idéntica al código anterior (480 de 480 cuadros); un recorte que excluye al avión lo pierde y uno que lo incluye lo recupera. Falta probarlo con la cámara real.
- **Por qué no es un zoom real:** cambiar el recorte o la escala del cuadro rompería la calibración y todo lo que depende de ella.
- **Con ventana de fondo:** elegir un solo vidrio, sin travesaños ni cortinas, con margen suficiente para que el avión no cruce el límite (en el borde su silueta queda cortada).

## 7. Trabajo con el repositorio

- Los cambios de código se suben **desde el PC**. La RPi sólo hace `git pull`: no tiene permiso de escritura en `palacioscb/G9_Capstone` y GitHub no acepta contraseña, sólo token.
- Lo que genera la RPi (perfiles, datos de medición) se trae al PC con `scp` y se sube desde ahí:
  ```bash
  scp g9-cap@g9-cap.local:~/G9_Capstone/raspberry_pi_5/rpi5_esqueleto/perfiles.json raspberry_pi_5/rpi5_esqueleto/
  ```
- `registros/` no va al repositorio (está en `.gitignore`). `perfiles.json` y `formas.json` sí.
- Antes de un `git pull` en la RPi, `git status` debe estar limpio. No mezclar en un mismo commit archivos de prueba (`A`, `consigna:`, `rtt_enlace.csv`).
- Si `git` dice que existe `.git/index.lock` y no hay otro git corriendo, es un archivo a medias: se borra (`del .git\index.lock` en Windows).

## Resultados en simulación (PC)

- **Detector rojo con una lámpara blanca en escena:** 160 de 160 cuadros, error de 0,41 px RMS (unos 0,05°, bajo el micropaso de 0,1125°). El detector por brillo se va a la lámpara en todos los cuadros (error de 254 px).
- **Detector por forma con plantilla aprendida** (escena sintética, avión de 12 × 7 cm): acierta el 100 % de los cuadros hasta 3 m con ventana y travesaños, con unos 3 ms por cuadro. A 4 m (61 px de área) ya no detecta. Esto es simulación: falta medirlo con el avión real a contraluz.
- **Modelo de pid_t_2:** reproduce los polos del PDF (pan −4,60 ± j141,8, −2,60, −2,41; tilt −58,3 ± j1365, −10,3, −9,75).
- **El PI del firmware compilado en C++ y su réplica en Python** dan la misma salida (|Δω| < 8·10⁻⁴ °/s en 3000 pasos, 1358 de ellos con límites activos).
- **Lazo cerrado simulado**, 10 s a 80 fps, objetivo de hasta 24,7°/s con dos oclusiones, ganancias de arranque y la ley final del firmware:

  | Configuración | Error de centrado RMS, todo SEGUIMIENTO | En régimen (0,6–3,2 s / 6,5–10 s) |
  |---|---|---|
  | PI puro, sin ω_ff | 31,6 px (3,6°) | — |
  | PI con ω_ff (`--feedforward`) | 11,1 px (1,3°) | 3,2 / 1,7 px (≈ 0,2–0,4°) |

  Con ω_ff, el error que queda está casi todo en la adquisición inicial y en la reaparición después de la oclusión de 0,5 s. Sin ω_ff, el PI sólo integra con |e| < 5°, así que se atrasa con un objetivo rápido.
- **Escalones (modelo con la ley final):** el peor sobrepaso es 2,3°, en escalones de 2 a 60°. Sin el tope de frenado y la banda integral llegaba a 42,7°.

## Modos reales del sensor (medido con `rpicam-hello --list-cameras`, 29-09-2026)

| Modo | Zona del sensor | fps máx | FOV H × V | °/px a 640 px |
|---|---|---|---|---|
| 640×480 | recorte 1280×960 | 200,16 | 28,2° × 21,4° | 0,045 |
| **1640×1232** (binning 2×2) | **completa** | **81,07** | **65,6° × 51,7°** | **0,115** |
| 1920×1080 | recorte 1920×1080 | 47,57 | 41,3° × 24,0° | 0,068 |
| 3280×2464 | completa | 21,19 | 65,6° × 51,7° | 0,115 |

`luz_op.py` y `laser_op.py` (en `mi_proyecto_camara`) no fijan el modo. Sus 120 fps y 5,9 ms son del modo recortado de 28°. Este programa fija 1640×1232.

## Pendiente

- Medir ζ (b) con `respuesta_escalon.py` y fijar las ganancias definitivas. El techo de estabilidad de pan depende de b: con ζ = 0,02 es Kp < 5,7.
- Calibrar f_px y las ganancias de color con el láser real. Medir el ruido de centroide y re-sintonizar λ.
- Aprender las vistas del avión real y ajustar el detector por forma a contraluz, con la ventana de fondo (sección 5). Pendiente de medir: distancia máxima útil, hojas y objetos detrás del avión, y el recorte con la cámara real.
- Patrón de búsqueda en BÚSQUEDA; finales de carrera; corrección de distorsión.
- La sección 5.1 del informe supone 30 fps; ahora son 80.
