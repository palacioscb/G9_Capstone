# Detección por forma: avión y objeto circular (18-11-2026)

Responde a la revisión de los ayudantes: el detector debe ser de imagen, no de luz. El avión
se reconoce por su figura, sin usar su color rojo ni canales RGB. Para el objeto no
identificado se usa el mismo método con una figura circular.

Código: `raspberry_pi_5/rpi5_esqueleto/pt/formas.py` (descriptores) y la clase `DetectorForma`
de `pt/deteccion.py`. Todo lo de este documento está medido con imágenes sintéticas; falta
validarlo con el avión real.

## 1. Qué cambió

| Detector (`--detector` o tablero) | Antes | Ahora |
|---|---|---|
| `avion` | Mancha roja en el plano V (color) | Silueta en el plano Y, reconocida por forma |
| `ovni` | No existía | Figura circular o elíptica en el plano Y |
| `avion_rojo` | — | El detector por color anterior, como respaldo |
| `rojo`, `luz`, `brillo` | Sin cambios | Sin cambios (el láser y el flash sí son puntos de luz) |

## 2. Método

1. **Región.** En SEGUIMIENTO, una ventana de 224 px alrededor de la detección anterior. En
   BÚSQUEDA, el cuadro completo a media resolución; lo que se encuentra se vuelve a medir a
   resolución completa.
2. **Segmentación por contraste local.** Realce morfológico sobre la luminancia: *black-hat*
   para objetos más oscuros que su entorno y *top-hat* para más claros. Mide cuánto se aparta
   cada píxel del fondo que lo rodea, así que no depende del nivel de luz ni de un degradado
   de iluminación. Sólo aparecen objetos más angostos que `tam_max_px`.
3. **Borde a media altura.** La figura se mide donde el contraste cae a la mitad de su
   máximo, que es donde está el borde real. Con el umbral bajo, las alas quedaban engrosadas.
4. **Soporte.** Una apertura proporcional al tamaño quita lo que es mucho más delgado que la
   figura (el palo), y después se devuelven las puntas de las alas.
5. **Normalización afín.** El contorno se "desaplasta" hasta que sus segundos momentos de
   área son iguales en todas las direcciones. Un objeto plano inclinado respecto de la
   cámara queda igual que visto de frente, salvo un giro. Toda elipse queda convertida en
   círculo.
6. **Descriptores de Fourier.** El contorno se recorre en 64 puntos equidistantes, cada punto
   como número complejo z = x + j·y, y se calcula su FFT. Se descarta F[0] (posición), se
   divide por |F[1]| (tamaño) y se usa sólo el módulo (giro y punto de partida). El descriptor
   son 15 números: los armónicos −8 a −1 y 2 a 8.
7. **Filtro de figura.** Distancia euclidiana entre el descriptor medido y cada plantilla
   pedida. Se acepta si es menor que `dist_max`. Además se filtra por área, alargamiento y
   salto entre cuadros.
8. **Posición.** Centroide de área del contorno, con resolución subpíxel.

Lectura de los armónicos: un círculo tiene toda su energía en F[1], así que su descriptor es
cero; un avión reparte energía en los armónicos altos, que describen alas y cola.

### Sobre la "librería que usa Fourier"

- Lo implementado usa `numpy.fft` directamente: el cálculo son unas 15 líneas y no agrega
  dependencias.
- `pyefd` (descriptores elípticos de Fourier, Kuhl y Giardina 1982) hace lo equivalente y se
  instala con `pip install pyefd`. No se usó para no agregar una dependencia a la RPi.
- OpenCV tiene `cv.ximgproc.fourierDescriptor` y `ContourFitting`, pero están en el módulo
  *contrib*, que no viene en todas las instalaciones (en el PC de pruebas no estaba).
- `cv.HoughCircles` detecta círculos por votación de bordes. No se usó porque sólo sirve para
  círculos (no para elipses ni para el avión) y es sensible a sus dos umbrales.

## 3. Resultados en escenas sintéticas

`python3 herramientas/evaluar_forma.py --n 150`. Escenas de 640×480 con degradado de
iluminación, ruido y un objetivo de 45 a 120 px con giro, inclinación y posición al azar.
Acierto: detección válida a menos de 5 px del objetivo.

| Caso | Acierto | Falso positivo | Error del centroide |
|---|---|---|---|
| Avión de planta, fondo liso | 98,7 % | 0,0 % | 0,61 px RMS |
| Avión claro sobre fondo oscuro (polaridad `claro`) | 95,3 % | 0,0 % | 0,64 px RMS |
| Avión con los niveles de la grabación real (50 de gris sobre fondo de 64 a 99) | 98,3 % | 0,0 % | 0,59 px RMS |
| Avión chico (25 a 45 px) | 57,3 % | 0,0 % | 0,68 px RMS |
| Avión entre rectángulos, barras, manchas y una lámpara | 98,7 % | 0,0 % | 0,57 px RMS |
| Avión con un palo de 2 a 4 px pegado | 57,3 % | 0,0 % | 1,44 px RMS |
| Avión de otro modelo, con la plantilla genérica | 0,0 % | 0,0 % | — |
| Avión de otro modelo, después de aprenderlo | 100 % | 0,0 % | — |
| Sin avión, sólo distractores | — | 1,3 % | — |
| Disco (círculo o elipse), fondo liso | 99,3 % | 0,0 % | 0,48 px RMS |
| Disco entre distractores, sin lámpara | 99,3 % | 0,0 % | 0,48 px RMS |
| Disco con una lámpara redonda en cuadro | 26,7 % | 73,3 % | — |
| Sin disco, sólo distractores | — | 2,7 % | — |

- **Seguimiento (400 cuadros, objetivo en movimiento):** 100 % de acierto sin distractores.
  Baja a 55 % si el objetivo pasa por delante de otras figuras del mismo tono: mientras se
  tocan, se funden en una sola mancha.
- **Tiempo de proceso (PC de pruebas):** 1,1 a 2,1 ms de mediana siguiendo y 1,7 a 3,9 ms
  buscando; el máximo observado fue 9,7 ms. El período de cuadro es 12,5 ms. Falta medirlo
  en la RPi 5.
- **Lazo cerrado simulado** (`main.py --enlace simulado --objeto avion --detector avion
  --feedforward`): error de detección 0,32 px RMS y error de centrado en régimen de 0,19°
  (1,7 px) en pan, igual que con el láser.

Ejemplos: `docs/figuras/deteccion_forma_ejemplos.png`.

## 4. Condiciones de operación (DET-6)

- El objetivo debe verse **completo**, con al menos unos 50 px de largo (área sobre 400 px).
- Debe **contrastar** con lo que lo rodea, y hay que decir si es más oscuro o más claro que
  el fondo (`polaridad`). Con `ambos` suben los falsos: en escenas con distractores, una
  figura ajena pasa el umbral en el 2 % de los cuadros con `oscuro` y en el 27 a 49 % con `ambos`.
- **Fondo liso** alrededor del objetivo. No debe tocar otra figura de contraste parecido.
- **Soporte:** un palo del mismo tono que el avión arruina la silueta. Usar hilo, un soporte
  delgado (menos de 2 px en la imagen) o del color del fondo.
- **Una plantilla por cara.** La inclinación de un objeto plano se compensa; el cambio de
  cara (de arriba a de lado) no. Las vistas muy delgadas (más de 6 a 1) se descartan.
- **La plantilla incorporada es genérica** (un avión de pasajeros visto desde arriba). Con
  otro modelo de avión no sirve: hay que aprender las vistas del avión real (sección 5). La
  genérica de perfil se retiró del banco: ver sección 6.
- **El detector circular no distingue un disco de una lámpara redonda.** Es el límite de
  decidir sólo por la figura.
- **Exposición:** la imagen debe mostrar el objeto, no sólo las luces. Punto de partida:
  6000 µs y ganancia 2,0.

## 5. Puesta en marcha con el avión real

```bash
# 1. Aprender cada cara del avión (objetivo solo, centrado, fondo liso, sin el palo a la vista)
python3 herramientas/aprender_forma.py --nombre avion_arriba --camara --exposicion 6000 --ganancia 2
python3 herramientas/aprender_forma.py --nombre avion_lado   --camara --exposicion 6000 --ganancia 2
python3 herramientas/aprender_forma.py --listar

# 2. Probar sin motores, mirando el tablero
python3 main.py --fuente camara --detector avion --monitor --duracion 0
```

- `aprender_forma.py` guarda las plantillas en `formas.json`, imprime la estabilidad de la
  figura entre cuadros con el `dist_max` recomendado, y deja en `registros/` una imagen con
  el contorno medido para revisarlo.
- Las vistas aprendidas reemplazan a las genéricas de su familia (`avion_*`).
- En el tablero, grupo Detector: polaridad, contraste mínimo, parecido exigido (`dist_max`),
  ancho máximo y área mínima. El panel Detección muestra la plantilla elegida y su distancia.
- El CSV tiene las columnas `forma` y `forma_dist`, y `motivo` dice por qué se rechazó un cuadro.

## 6. Primera grabación con el avión real (18-11-2026)

`mi_proyecto_camara/medicion_avion/` (90 s, 360 cuadros guardados, 8000 µs, ganancia 4,0).

**No sirve para aprender la silueta: el avión no aparece completo en ningún cuadro.** Se
grabó muy cerca: ocupa entre 265 y 590 px de los 640 del cuadro y toca el borde en los 317
cuadros en que se ve. El detector necesita la figura entera.

Lo que sí se pudo medir:

| Medición | Valor |
|---|---|
| Nivel de gris del avión | 45 a 57 (es mate y oscuro) |
| Nivel del fondo vacío | 81 de media, con degradado de 64 a 99 dentro del cuadro |
| Contraste avión–fondo | 36 a 41 de media; 14 a 49 según la zona |
| Ruido por píxel | 2,4 niveles de gris |
| Contraste local del fondo vacío | hasta 17 en algún píxel; ninguna mancha de 150 px con umbral 12 |
| Saturación | 0 % |
| Tiempo del detector (PC, cuadros reales) | 1,0 a 1,3 ms de mediana |

Cambios hechos a partir de esos datos:

- **Polaridad `oscuro` confirmada:** el avión es más oscuro que el fondo en todo el cuadro.
- **`contraste_min` del avión: de 25 a 12.** Con 25, donde el fondo es más oscuro el avión
  quedaba bajo el umbral. En escenas sintéticas con esos mismos niveles, el acierto sube de
  43 % a 98 %; en los 24 cuadros reales sin avión no aparece ninguna detección.
- **Borde a media altura contra el fondo local**, píxel a píxel, en vez de contra el máximo de
  contraste de la figura: con un degradado de 35 niveles la regla anterior recortaba el avión.
- **Se retiró la plantilla genérica de perfil.** Aceptó como "avión" trozos sueltos del avión
  real (punta de ala, nariz) en 44 de los 336 cuadros con avión. Con la de planta sola: 0.
- **El avión real es de ala delta, con una sonda larga en la nariz.** La plantilla genérica
  de planta tampoco le corresponde: hay que aprender sus vistas.

Para la próxima grabación: avión completo en cuadro, de 100 a 200 px de largo. Con el campo
de la cámara (0,1154° por píxel, unos 2,0 mm por píxel a 1 m), un avión de 30 cm mide 150 px
a 1 m y 100 px a 1,5 m. Usar `herramientas/grabar_cuadros.py`, una etiqueta por cara.

## 7. Referencias

- Zahn, C. T. y Roskies, R. Z. (1972). Fourier descriptors for plane closed curves. *IEEE Trans. Computers*.
- Persoon, E. y Fu, K. S. (1977). Shape discrimination using Fourier descriptors. *IEEE Trans. SMC*.
- Kuhl, F. P. y Giardina, C. R. (1982). Elliptic Fourier features of a closed contour. *Computer Graphics and Image Processing* 18, 236–258.
- Arbter, K. et al. (1990). Application of affine-invariant Fourier descriptors to recognition of 3-D objects. *IEEE Trans. PAMI* 12(7).
- Dalitz, C. et al. (2013). Fourier descriptors for broken shapes. *EURASIP J. Adv. Signal Process.* https://asp-eurasipjournals.springeropen.com/articles/10.1186/1687-6180-2013-161
- PyEFD: https://pyefd.readthedocs.io/
- OpenCV, descriptores de Fourier (ximgproc): https://docs.opencv.org/3.4/dd/ddc/group__ximgproc__fourier.html
- OpenCV, transformada de Hough para círculos: https://docs.opencv.org/5.0/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html
