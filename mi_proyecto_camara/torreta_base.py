import os
import time
from datetime import datetime
import cv2
import numpy as np
from picamera2 import Picamera2

# 1. PARÁMETROS DE CONFIGURACIÓN
WIDTH, HEIGHT = 1280, 720
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
UMBRAL_BRILLO = 220  # Ajustar sensibilidad (0 - 255)

# Crear carpeta para guardar las fotos si no existe
DIR_CAPTURAS = "capturas"
os.makedirs(DIR_CAPTURAS, exist_ok=True)

# 2. INICIALIZAR CAMARA
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

# Fijar tasa a 60 FPS estables
picam2.set_controls({"FrameDurationLimits": (16666, 16666)})

# 3. CONFIGURACIÓN DE VENTANA EN PANTALLA HDMI
cv2.namedWindow("Torreta - Control de Vision", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Torreta - Control de Vision", 1280, 720)

print(f"[OK] Sistema iniciado a {WIDTH}x{HEIGHT}.")
print(f"[OK] Las capturas se guardarán en: ./{DIR_CAPTURAS}/")
print("Controles: Presiona 's' para tomar foto | 'q' para salir.")

prev_time = time.perf_counter()
contador_fotos = 0

try:
    while True:
        # Captura y conversión de color
        frame_rgb = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # Detección del punto más brillante
        _, max_val, _, max_loc = cv2.minMaxLoc(gray)
        target_x, target_y = max_loc

        # Dibujar centro objetivo (Cruz azul)
        cv2.drawMarker(frame_bgr, (CENTER_X, CENTER_Y), (255, 0, 0), cv2.MARKER_CROSS, 15, 1)

        # Evaluar detección
        if max_val >= UMBRAL_BRILLO:
            error_x = target_x - CENTER_X
            error_y = target_y - CENTER_Y
            
            # Dibujar objetivo (Círculo verde) y vector de error
            cv2.circle(frame_bgr, (target_x, target_y), 6, (0, 255, 0), 2)
            cv2.line(frame_bgr, (CENTER_X, CENTER_Y), (target_x, target_y), (0, 255, 255), 1)
            txt_estado = f"Laser dX:{error_x} dY:{error_y}"
        else:
            txt_estado = "Buscando objetivo..."

        # Cálculo de FPS
        curr_time = time.perf_counter()
        fps = 1.0 / (curr_time - prev_time)
        prev_time = curr_time

        # Superponer texto en pantalla
        cv2.putText(frame_bgr, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame_bgr, txt_estado, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame_bgr, "[S] Foto  |  [Q] Salir", (10, HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Mostrar ventana en el monitor HDMI
        cv2.imshow("Torreta - Control de Vision", frame_bgr)

        # Lectura de teclado
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            contador_fotos += 1
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(DIR_CAPTURAS, f"captura_{contador_fotos}_{timestamp}.jpg")
            
            # Guardar la imagen con las marcas y datos visibles
            cv2.imwrite(filename, frame_bgr)
            print(f"[FOTO {contador_fotos}] Guardada en: {filename}")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
