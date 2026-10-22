# poc_drv8825 — proyecto PlatformIO

Estructura:
- `platformio.ini` — configuración (placa `esp32dev`, framework Arduino, monitor a 115200 con fin de línea LF).
- `src/main.cpp` — firmware: generador STEP/DIR con rampa trapezoidal y comandos por serial.
- `protocolo_poc_drv8825.md` — protocolo de banco y tablas de registro (pruebas A–F).
- `conexiones_poc.png` — diagrama de conexiones ESP32 ↔ DRV8825 ↔ motor ↔ fuente.
- `arduino_ide/poc_drv8825/poc_drv8825.ino` — el mismo código para quien use Arduino IDE.

Uso en VSCode:
1. Instalar la extensión "PlatformIO IDE". Reiniciar VSCode.
2. PIO Home → Open Project → seleccionar esta carpeta (`pruebas/poc_drv8825`).
3. Conectar el ESP32 por USB. Barra inferior: ✓ (Build) → → (Upload) → 🔌 (Serial Monitor).
   Si el upload no encuentra el puerto, agregar `upload_port = COMx` en `platformio.ini`.
4. En el monitor escribir `?` + Enter para ver los comandos. El driver parte deshabilitado (`e 1` para habilitar).

Si la placa no es un ESP32 clásico (p. ej. ESP32-S3 o C3), cambiar `board` en `platformio.ini`
y revisar que los pines usados existan (`PIN_*` al inicio de `main.cpp`). El acceso directo
`GPIO.out_w1ts` requiere que `PIN_STEP` sea un GPIO < 32.
