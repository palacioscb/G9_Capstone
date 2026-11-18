# Detección por forma: avión y objeto circular (18-11-2026)

Responde a la revisión de los ayudantes: el detector debe ser de imagen, no de luz. El avión
se reconoce por su figura, sin usar su color ni canales RGB. Para el objeto no identificado
se usa el mismo método con una figura circular.

Código: `raspberry_pi_5/rpi5_esqueleto/pt/formas.py` (descriptores) y la clase `DetectorForma`
de `pt/deteccion.py`. Lo medido en este documento sale de fotos del avión real tomadas con
celular; falta repetirlo con la cámara de la RPi.

## 1. Qué cambió

| Detector (`--detector` o tablero) | Antes | Ahora |
|---|---|---|
| `avion` | Mancha roja en el plano V (color) | Silueta en el plano Y, reconocida por forma |
| `ovni` | No existía | Figura circular o elíptica en el plano Y |
| `rojo`, `luz`, `brillo` | Sin cambios | Sin cambios (el láser y el flash sí son puntos de luz) |

Retirado del repositorio el 18-11-2026, por corresponder a datos o supuestos anteriores:

- el detector del avión por color rojo (`avion_rojo`);
- las siluetas genéricas de avión que venían incorporadas y las escenas sintéticas que las usaban;
- la grabación `medicion_avion` y el programa que la generaba.

Las vistas del avión no vienen incorporadas: se aprenden del avión real y quedan en `formas.json`.

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

## 3. Resultados con el avión real (fotos de celular, 18-11-2026)

Fotos del avión sobre una mesa lisa, llevadas a 640×480 en gris con el avión de unos 150 px
de largo (`herramientas/preparar_fotos.py`). Cinco fotos desde arriba, cinco de lado y tres
de un fondo sin avión (ventana con árboles). Se reproduce con:

```bash
python3 herramientas/reproducir_forma.py registros/capturas/foto_arriba registros/capturas/foto_lado registros/capturas/sin_avion_foto
```

| Conjunto | Reconocidas | Distancia a la plantilla |
|---|---|---|
| Avión desde arriba (`avion_arriba`) | 5 de 5 | 0,04 a 0,08 |
| Avión de lado (`avion_lado`) | 5 de 5 | 0,11 a 0,18 |
| Fondo sin avión | 0 de 3 | la figura más parecida queda en 0,21 |

- **Las dos vistas no se confunden:** las plantillas quedan a 0,36 una de la otra.
- **`dist_max` = 0,18.** Con 0,15 se pierde una de las cinco fotos de lado. El margen frente
  al fondo (0,21) es estrecho.
- **`alargamiento_max` = 8.** De lado, el avión mide 4,7 a 5,7 de largo/ancho; con el límite
  general de 6 quedaba al borde. Con 8 no aparece ninguna detección en las fotos de fondo.
- **Sombra.** Con el avión apoyado, su sombra se pega a la silueta de lado. La vista de lado
  se aprendió con el borde más adentro (`aprender_forma.py --frac-borde 0.8`), que deja la
  sombra fuera.
- **Tiempo de proceso (PC):** 2,5 a 2,8 ms por foto buscando en el cuadro completo. El período
  de cuadro es 12,5 ms. Falta medirlo en la RPi 5.

Límites de estos resultados:

- Las plantillas se aprendieron con estas mismas fotos, así que las distancias son optimistas.
  Dejando una foto fuera: 0,07 a 0,09 desde arriba y 0,04 a 0,11 de lado.
- Son fotos de celular sobre una mesa. Falta el avión frente a la cámara de la RPi, en el aire
  y a la distancia de la demostración.
- No se probó el avión delante del fondo con árboles.
- El detector circular (`ovni`) no se ha probado con el objeto real: sólo lo cubren las pruebas
  unitarias, con un disco dibujado.

## 4. Condiciones de operación (DET-6)

- El objetivo debe verse **completo**, de 100 a 200 px de largo. Con el campo de la cámara
  (0,1154° por píxel, unos 2,0 mm por píxel a 1 m), un avión de 30 cm mide 150 px a 1 m y
  100 px a 1,5 m. Más grande que unos 200 px, el detector sólo ve sus partes delgadas.
- Debe **contrastar** con lo que lo rodea, y hay que decir si es más oscuro o más claro que
  el fondo (`polaridad`). Con `ambos` suben los falsos: el fondo claro encerrado entre el ala
  y la cola cuenta como una figura más.
- **Fondo liso** alrededor del objetivo. No debe tocar otra figura de contraste parecido.
- **Soporte:** un palo del mismo tono que el avión arruina la silueta. Usar hilo, un soporte
  delgado (menos de 2 px en la imagen) o del color del fondo.
- **Una plantilla por cara.** La inclinación de un objeto plano se compensa; el cambio de
  cara (de arriba a de lado) no. Las vistas muy delgadas (más de 8 a 1) se descartan.
- **Sin vistas aprendidas no hay detección.** El tablero lo avisa: "sin plantillas de avion".
- **El detector circular no distingue un disco de una lámpara redonda.** Es el límite de
  decidir sólo por la figura.
- **Exposición:** la imagen debe mostrar el objeto, no sólo las luces. El perfil `avion` parte
  con 8000 µs y ganancia 4,0, que son valores de partida sin medición vigente: ajustarlos con
  el avión frente a la cámara y regrabar el perfil.

## 5. Puesta en marcha con el avión real

```bash
# 1. Aprender cada cara del avión (objetivo solo, centrado, fondo liso, sin el palo a la vista)
python3 herramientas/aprender_forma.py --nombre avion_arriba --camara --exposicion 8000 --ganancia 4
python3 herramientas/aprender_forma.py --nombre avion_lado   --camara --exposicion 8000 --ganancia 4
python3 herramientas/aprender_forma.py --listar

# 2. Probar sin motores, mirando el tablero (aplica solo el perfil «avion»)
python3 main.py --fuente camara --detector avion --monitor --duracion 0
```

- `aprender_forma.py` guarda las plantillas en `formas.json`, imprime la estabilidad de la
  figura entre cuadros con el `dist_max` recomendado, y deja en `registros/` una imagen con
  el contorno medido para revisarlo.
- También aprende desde fotos: `--imagen "registros/capturas/foto_arriba/*.png"`.
- En el tablero, grupo Detector: polaridad, contraste mínimo, parecido exigido (`dist_max`),
  ancho máximo, área mínima y alargamiento máximo. El panel Detección muestra la plantilla
  elegida y su distancia.
- El CSV tiene las columnas `forma` y `forma_dist`, y `motivo` dice por qué se rechazó un cuadro.

## 6. Referencias

- Zahn, C. T. y Roskies, R. Z. (1972). Fourier descriptors for plane closed curves. *IEEE Trans. Computers*.
- Persoon, E. y Fu, K. S. (1977). Shape discrimination using Fourier descriptors. *IEEE Trans. SMC*.
- Kuhl, F. P. y Giardina, C. R. (1982). Elliptic Fourier features of a closed contour. *Computer Graphics and Image Processing* 18, 236–258.
- Arbter, K. et al. (1990). Application of affine-invariant Fourier descriptors to recognition of 3-D objects. *IEEE Trans. PAMI* 12(7).
- Dalitz, C. et al. (2013). Fourier descriptors for broken shapes. *EURASIP J. Adv. Signal Process.* https://asp-eurasipjournals.springeropen.com/articles/10.1186/1687-6180-2013-161
- PyEFD: https://pyefd.readthedocs.io/
- OpenCV, descriptores de Fourier (ximgproc): https://docs.opencv.org/3.4/dd/ddc/group__ximgproc__fourier.html
- OpenCV, transformada de Hough para círculos: https://docs.opencv.org/5.0/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html
