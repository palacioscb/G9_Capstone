#!/usr/bin/env python3
"""
generar_figuras.py — Figuras del documento de conexiones de laboratorio (IEE2913, Grupo 9).

  fig_pines_esp32.png      mapa de pines del ESP32 DevKit con la asignación del firmware
  fig_conexiones_eje.png   cableado de UN eje (driver, motor, sensor, topes, fuente)

#! Los números de pin salen de esp_32/esp32_pan_tilt/include/config.h. Si se cambian allá,
#! regenerar: python3 docs/figuras/generar_figuras.py
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

AQUI = os.path.dirname(os.path.abspath(__file__))

# Colores por grupo
C_PAN, C_TILT, C_COMP = "#1f6fb4", "#d9730d", "#7b3fa0"
C_POT, C_GND, C_LIBRE, C_NO = "#c62828", "#333333", "#9e9e9e", "#bdbdbd"
C_SENS = "#2e7d32"


# ---------------------------------------------------------------------------
#* Figura 1: mapa de pines (ESP32-DevKitC de 38 pines, USB abajo)
# ---------------------------------------------------------------------------
IZQ = ["3V3", "EN", "36", "39", "34", "35", "32", "33", "25", "26", "27", "14", "12", "GND", "13",
       "D2", "D3", "CMD", "5V"]
DER = ["GND", "23", "22", "TX", "RX", "21", "GND", "19", "18", "5", "17", "16", "4", "0", "2", "15",
       "D1", "D0", "CLK"]

ASIG = {   # pin: (texto, color)
    "34": ("nFAULT pan  (pull-up ext. 10 kΩ)", C_PAN),
    "35": ("nFAULT tilt (pull-up ext. 10 kΩ)", C_TILT),
    "32": ("M0  (ambos drivers)", C_COMP),
    "33": ("M1  (ambos drivers)", C_COMP),
    "25": ("STEP pan", C_PAN),
    "26": ("DIR pan", C_PAN),
    "27": ("nENABLE pan (pull-up ext. 4,7 kΩ)", C_PAN),
    "14": ("M2  (ambos drivers)", C_COMP),
    "13": ("fin de carrera pan (2 NC en serie)", C_PAN),
    "23": ("fin de carrera tilt (2 NC en serie)", C_TILT),
    "22": ("SCL AS5600 pan (I2C 0)", C_PAN),
    "21": ("SDA AS5600 pan (I2C 0)", C_PAN),
    "19": ("SCL AS5600 tilt (I2C 1)", C_TILT),
    "18": ("SDA AS5600 tilt (I2C 1)", C_TILT),
    "17": ("STEP tilt", C_TILT),
    "16": ("DIR tilt", C_TILT),
    "4":  ("nENABLE tilt (pull-up ext. 4,7 kΩ)", C_TILT),
    "3V3": ("3V3: lógica DRV8825, AS5600, pull-ups", C_POT),
    "GND": ("GND común", C_GND),
    "36": ("reservado: ADC shunt pan", C_LIBRE),
    "39": ("reservado: ADC shunt tilt", C_LIBRE),
    "TX": ("USB a la RPi (UART0)", C_LIBRE),
    "RX": ("USB a la RPi (UART0)", C_LIBRE),
}
NO_USAR = {"EN", "12", "D2", "D3", "CMD", "5V", "5", "0", "2", "15", "D1", "D0", "CLK"}


def fig_pines():
    fig, ax = plt.subplots(figsize=(11.5, 8.6))
    ax.set_xlim(0, 11.5); ax.set_ylim(-0.6, 20.6); ax.axis("off")
    x0, x1 = 4.55, 6.95                       # bordes de la placa
    ax.add_patch(FancyBboxPatch((x0, 0.3), x1 - x0, 19.6, boxstyle="round,pad=0.05",
                                fc="#f2f2f2", ec="black", lw=1.5))
    ax.add_patch(Rectangle((5.25, 0.0), 1.0, 0.7, fc="#cfcfcf", ec="black"))
    ax.text(5.75, 0.35, "USB", ha="center", va="center", fontsize=8)
    ax.add_patch(Rectangle((5.35, 15.6), 0.8, 3.4, fc="#dcdcdc", ec="black"))
    ax.text(5.75, 17.3, "ESP32\nWROOM\n-32", ha="center", va="center", fontsize=6.5)
    ax.text(5.75, 20.35, "ESP32-DevKitC (38 pines), vista superior", ha="center", fontsize=11, weight="bold")

    def fila(i):
        return 19.2 - i * 1.0

    for i, pin in enumerate(IZQ):
        y = fila(i)
        txt, col = ASIG.get(pin, ("no usar (arranque/flash)" if pin in NO_USAR else "libre", C_NO))
        ax.add_patch(Rectangle((x0 - 0.05, y - 0.2), 0.4, 0.4, fc=col, ec="black", lw=0.6))
        ax.text(x0 + 0.45, y, pin, va="center", ha="left", fontsize=8, family="monospace")
        ax.plot([x0 - 0.05, x0 - 0.35], [y, y], color=col, lw=2)
        ax.text(x0 - 0.45, y, txt, va="center", ha="right", fontsize=8.6, color=col,
                weight="bold" if col not in (C_NO, C_LIBRE) else "normal")
    for i, pin in enumerate(DER):
        y = fila(i)
        txt, col = ASIG.get(pin, ("no usar (arranque/flash)" if pin in NO_USAR else "libre", C_NO))
        ax.add_patch(Rectangle((x1 - 0.35, y - 0.2), 0.4, 0.4, fc=col, ec="black", lw=0.6))
        ax.text(x1 - 0.45, y, pin, va="center", ha="right", fontsize=8, family="monospace")
        ax.plot([x1 + 0.05, x1 + 0.35], [y, y], color=col, lw=2)
        ax.text(x1 + 0.45, y, txt, va="center", ha="left", fontsize=8.6, color=col,
                weight="bold" if col not in (C_NO, C_LIBRE) else "normal")

    ley = [(C_PAN, "eje PAN (azimut)"), (C_TILT, "eje TILT (elevación)"), (C_COMP, "compartido por ambos drivers"),
           (C_POT, "alimentación lógica 3V3"), (C_GND, "tierra"), (C_LIBRE, "reservado / USB"), (C_NO, "no usar")]
    for k, (c, t) in enumerate(ley):
        ax.add_patch(Rectangle((0.1, 2.4 - k * 0.42), 0.3, 0.3, fc=c, ec="black", lw=0.5))
        ax.text(0.5, 2.55 - k * 0.42, t, va="center", fontsize=8)
    ax.text(11.4, -0.4, "En un DevKit de 30 pines el orden de los GPIO es el mismo; guiarse por la serigrafía.",
            ha="right", fontsize=7.5, style="italic")
    fig.tight_layout()
    fig.savefig(os.path.join(AQUI, "fig_pines_esp32.png"), dpi=170)
    plt.close(fig)


# ---------------------------------------------------------------------------
#* Figura 2: cableado de un eje
# ---------------------------------------------------------------------------
def caja(ax, x, y, w, h, titulo, color="#eef3fb"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.04", fc=color, ec="black", lw=1.3))
    ax.text(x + w / 2, y + h + 0.18, titulo, ha="center", va="bottom", fontsize=10, weight="bold")


def pin(ax, x, y, texto, lado="izq"):
    ax.add_patch(Rectangle((x - 0.12, y - 0.12), 0.24, 0.24, fc="white", ec="black", lw=0.8))
    if lado == "izq":
        ax.text(x + 0.22, y, texto, va="center", ha="left", fontsize=8, family="monospace")
    else:
        ax.text(x - 0.22, y, texto, va="center", ha="right", fontsize=8, family="monospace")


def cable(ax, puntos, color, lw=1.8, ls="-"):
    xs, ys = zip(*puntos)
    ax.plot(xs, ys, color=color, lw=lw, ls=ls, solid_capstyle="round")


def fig_eje():
    fig, ax = plt.subplots(figsize=(13, 8.6))
    ax.set_xlim(0, 13); ax.set_ylim(0, 8.8); ax.axis("off")
    ax.text(6.5, 8.55, "Cableado de UN eje — pan (repetir para tilt con los pines de la columna derecha)",
            ha="center", fontsize=12, weight="bold")

    # ESP32
    caja(ax, 0.3, 1.0, 2.4, 6.4, "ESP32 DevKit", "#f4f4f4")
    ep = {"GPIO25 (17)": 6.9, "GPIO26 (16)": 6.4, "GPIO27 (4)": 5.9, "GPIO34 (35)": 5.4,
          "GPIO32/33/14": 4.9, "GPIO21 (18)": 3.9, "GPIO22 (19)": 3.4, "GPIO13 (23)": 2.4,
          "3V3": 1.9, "GND": 1.4}
    for t, y in ep.items():
        pin(ax, 2.7, y, t, "der")

    # DRV8825
    caja(ax, 4.6, 3.95, 2.6, 3.45, "DRV8825 (Pololu)")
    dp_izq = {"STEP": 6.9, "DIR": 6.4, "ENABLE": 5.9, "FAULT": 5.4, "M0 M1 M2": 4.9, "RST+SLP": 4.4}
    for t, y in dp_izq.items():
        pin(ax, 4.6, y, t, "izq")
    dp_der = {"VMOT": 6.9, "GND": 6.4, "B2": 5.9, "B1": 5.4, "A1": 4.9, "A2": 4.4}
    for t, y in dp_der.items():
        pin(ax, 7.2, y, t, "der")
    ax.text(5.9, 4.1, "Vref: pot. del módulo", ha="center", fontsize=7.5, style="italic")

    # Señales lógicas
    for t, yd in (("STEP", 6.9), ("DIR", 6.4), ("ENABLE", 5.9), ("FAULT", 5.4), ("M0 M1 M2", 4.9)):
        cable(ax, [(2.82, yd), (4.48, yd)], C_PAN)
    ax.text(3.65, 7.0, "100 Ω serie (opc.)", ha="center", fontsize=7)
    cable(ax, [(2.82, 1.9), (3.3, 1.9), (3.3, 4.4), (4.48, 4.4)], C_POT)
    # pull-ups
    for yv, txt in ((5.9, "pull-up 4,7 kΩ a 3V3"), (5.4, "pull-up 10 kΩ a 3V3")):
        cable(ax, [(4.15, yv), (4.15, yv + 0.2)], "#8a6d00", 1.4)
        ax.add_patch(Rectangle((4.05, yv + 0.2), 0.2, 0.08, fc="#fff3c4", ec="black", lw=0.8))
        ax.text(4.0, yv + 0.24, txt, ha="right", va="center", fontsize=6.8, color="#8a6d00")

    # Motor: cada bobina en línea recta, sin cruces
    import math as _m
    cxm, cym, rm = 9.75, 5.15, 0.95
    ax.add_patch(plt.Circle((cxm, cym), rm, fc="#e8f5e9", ec="black", lw=1.3))
    ax.text(cxm + 0.1, cym, "NEMA 17\nbipolar", ha="center", va="center", fontsize=9, weight="bold")
    for t, y in (("B2", 5.9), ("B1", 5.4), ("A1", 4.9), ("A2", 4.4)):
        xb = cxm - _m.sqrt(max(rm ** 2 - (y - cym) ** 2, 0.0))
        cable(ax, [(7.32, y), (xb, y)], C_SENS)
        ax.text(xb + 0.08, y, t, va="center", fontsize=7, family="monospace")
    ax.text(cxm, 3.85, "bobina A: A1–A2 · bobina B: B1–B2\n(pocos Ω entre pares)", ha="center", fontsize=7.5)

    # Fuente
    ax.add_patch(FancyBboxPatch((10.6, 6.6), 2.1, 1.1, boxstyle="round,pad=0.04", fc="#fdecea", ec="black", lw=1.3))
    ax.text(11.65, 7.3, "Fuente 12 V", ha="center", fontsize=10, weight="bold")
    ax.text(11.65, 6.9, "límite 1,0 A al partir", ha="center", fontsize=7.5)
    cable(ax, [(7.32, 6.9), (7.9, 6.9), (7.9, 7.9), (11.1, 7.9), (11.1, 7.7)], C_POT, 2.2)
    cable(ax, [(7.32, 6.4), (7.6, 6.4), (7.6, 8.1), (12.2, 8.1), (12.2, 7.7)], C_GND, 2.2)
    ax.text(9.5, 8.18, "+12 V a VMOT   ·   GND", ha="center", fontsize=8)
    ax.text(8.2, 7.45, "≥ 100 µF / 25 V\njunto al módulo", ha="left", fontsize=7.5, color=C_POT)

    # AS5600
    caja(ax, 4.6, 1.85, 2.6, 1.55, "AS5600 (en el EJE)", "#eaf6ea")
    for t, y in (("SDA", 3.05), ("SCL", 2.65), ("VCC 3V3", 2.25)):
        pin(ax, 4.6, y, t, "izq")
    for t, y in (("GND", 3.05), ("DIR→GND", 2.65)):
        pin(ax, 7.2, y, t, "der")
    cable(ax, [(2.82, 3.9), (3.9, 3.9), (3.9, 3.05), (4.48, 3.05)], C_SENS)
    cable(ax, [(2.82, 3.4), (3.7, 3.4), (3.7, 2.65), (4.48, 2.65)], C_SENS)
    cable(ax, [(3.3, 1.9), (3.3, 2.25), (4.48, 2.25)], C_POT)
    ax.text(9.0, 2.75, "imán diametral centrado en el eje,\n0,5–3 mm sobre el chip", fontsize=7.5)

    # Topes
    caja(ax, 4.6, 0.25, 2.6, 1.0, "Topes de carrera (2, NC, en serie)", "#f3eefa")
    ax.text(5.9, 0.75, "GPIO ─ NC₁ ─ NC₂ ─ GND", ha="center", fontsize=8.5, family="monospace")
    cable(ax, [(2.82, 2.4), (3.5, 2.4), (3.5, 0.75), (4.48, 0.75)], C_COMP)

    # GND común
    cable(ax, [(2.82, 1.4), (2.95, 1.4), (2.95, 0.15), (8.6, 0.15), (8.6, 3.05), (7.32, 3.05)], C_GND, 1.6, "--")
    cable(ax, [(7.32, 2.65), (8.6, 2.65)], C_GND, 1.6, "--")
    cable(ax, [(7.2, 0.75), (8.6, 0.75)], C_GND, 1.6, "--")
    ax.text(10.4, 0.6, "GND común: ESP32 – DRV8825 – AS5600 –\nfuente de 12 V – topes", ha="center",
            fontsize=8, weight="bold")

    tabla = ("Pines tilt (entre paréntesis): STEP 17, DIR 16, nEN 4, nFAULT 35,\n"
             "SDA 18, SCL 19, fin de carrera 23. M0/M1/M2 (32/33/14) son comunes.")
    ax.text(10.5, 1.6, tabla, ha="center", fontsize=8, bbox=dict(fc="white", ec="#999"))
    fig.tight_layout()
    fig.savefig(os.path.join(AQUI, "fig_conexiones_eje.png"), dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    fig_pines()
    fig_eje()
    print("Figuras en", AQUI)
