#!/usr/bin/env python3
"""
md_a_pdf.py — Convierte docs/conexiones_laboratorio.md en PDF (reportlab).

Uso:  python3 docs/md_a_pdf.py [entrada.md] [salida.pdf]
Cubre lo que usa el documento: títulos, párrafos, listas (con un nivel de anidación),
listas numeradas, tablas, bloques de código, imágenes, **negrita** y `código`.
"""
import html
import os
import re
import sys

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, KeepTogether, ListFlowable, ListItem, Paragraph, Preformatted,
                                SimpleDocTemplate, Spacer, Table, TableStyle)

F = "/usr/share/fonts/truetype/dejavu/"
pdfmetrics.registerFont(TTFont("DV", F + "DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("DV-B", F + "DejaVuSans-Bold.ttf"))
pdfmetrics.registerFont(TTFont("DV-I", F + "DejaVuSans-Oblique.ttf"))
pdfmetrics.registerFont(TTFont("DVM", F + "DejaVuSansMono.ttf"))
pdfmetrics.registerFontFamily("DV", normal="DV", bold="DV-B", italic="DV-I", boldItalic="DV-B")

AZUL = colors.HexColor("#1f4e79")
E = {
    "titulo": ParagraphStyle("t", fontName="DV-B", fontSize=16, leading=20, textColor=AZUL, spaceAfter=4),
    "sub": ParagraphStyle("s", fontName="DV-I", fontSize=9, leading=12, textColor=colors.grey, spaceAfter=10),
    "h2": ParagraphStyle("h2", fontName="DV-B", fontSize=12.5, leading=16, textColor=AZUL, spaceBefore=12, spaceAfter=5, keepWithNext=1),
    "h3": ParagraphStyle("h3", fontName="DV-B", fontSize=10.5, leading=14, spaceBefore=8, spaceAfter=4, keepWithNext=1),
    "p": ParagraphStyle("p", fontName="DV", fontSize=9.3, leading=12.8, spaceAfter=5, alignment=TA_LEFT),
    "celda": ParagraphStyle("c", fontName="DV", fontSize=7.8, leading=9.6),
    "celda_b": ParagraphStyle("cb", fontName="DV-B", fontSize=7.8, leading=9.6, textColor=colors.white),
    "cod": ParagraphStyle("cod", fontName="DVM", fontSize=7.6, leading=9.8, backColor=colors.HexColor("#f3f3f3"),
                          borderPadding=5, spaceBefore=3, spaceAfter=8),
}


def inline(t):
    """Markdown en línea -> mini-HTML de reportlab."""
    t = t.replace("\\|", "|")
    partes = re.split(r"(`[^`]+`)", t)
    out = []
    for p in partes:
        if p.startswith("`") and p.endswith("`"):
            out.append(f'<font name="DVM" size="8">{html.escape(p[1:-1])}</font>')
        else:
            p = html.escape(p)
            p = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", p)
            out.append(p)
    return "".join(out)


def tabla(filas, ancho):
    enc = [Paragraph(inline(c), E["celda_b"]) for c in filas[0]]
    cuerpo = [[Paragraph(inline(c), E["celda"]) for c in f] for f in filas[1:]]
    n = len(filas[0])
    largos = [max(len(f[i]) if i < len(f) else 0 for f in filas) for i in range(n)]
    pesos = [min(max(l, 10), 60) for l in largos]
    anchos = [ancho * p / sum(pesos) for p in pesos]
    t = Table([enc] + cuerpo, colWidths=anchos, repeatRows=1)
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), AZUL),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#eef3f9")]),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#b0b8c4")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3),
        ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2),
    ]))
    return t


def celdas(linea):
    linea = linea.strip().strip("|")
    return [c.strip() for c in re.split(r"(?<!\\)\|", linea)]


def convertir(md, base, ancho):
    lineas = md.split("\n")
    story, i = [], 0
    primero_h1 = True
    while i < len(lineas):
        l = lineas[i]
        if not l.strip():
            i += 1
            continue
        if l.startswith("# "):
            story.append(Paragraph(inline(l[2:]), E["titulo"]))
            if primero_h1 and i + 2 < len(lineas) and lineas[i + 2].strip() and not lineas[i + 2].startswith("#"):
                story.append(Paragraph(inline(lineas[i + 2]), E["sub"]))
                i += 3
                primero_h1 = False
                continue
            i += 1
        elif l.startswith("## "):
            story.append(Paragraph(inline(l[3:]), E["h2"])); i += 1
        elif l.startswith("### "):
            story.append(Paragraph(inline(l[4:]), E["h3"])); i += 1
        elif l.startswith("```"):
            j = i + 1
            while not lineas[j].startswith("```"):
                j += 1
            story.append(Preformatted("\n".join(lineas[i + 1:j]), E["cod"]))
            i = j + 1
        elif l.startswith("!["):
            m = re.match(r"!\[(.*?)\]\((.*?)\)", l)
            ruta = os.path.join(base, m.group(2))
            img = Image(ruta)
            esc = ancho / img.imageWidth
            img.drawWidth, img.drawHeight = ancho, img.imageHeight * esc
            if img.drawHeight > 17 * cm:
                f = 17 * cm / img.drawHeight
                img.drawWidth, img.drawHeight = img.drawWidth * f, img.drawHeight * f
            story.append(KeepTogether([img, Paragraph(f"<i>{inline(m.group(1))}</i>", E["sub"])]))
            i += 1
        elif l.startswith("|"):
            filas = []
            while i < len(lineas) and lineas[i].startswith("|"):
                if not re.match(r"^\|[\s\-:|]+\|$", lineas[i].strip()):
                    filas.append(celdas(lineas[i]))
                i += 1
            story.append(tabla(filas, ancho))
            story.append(Spacer(1, 6))
        elif re.match(r"^(\d+)\. ", l) or l.startswith("- "):
            numerada = bool(re.match(r"^\d+\. ", l))
            items = []
            while i < len(lineas) and (re.match(r"^(\d+)\. ", lineas[i]) or lineas[i].startswith("- ")
                                       or lineas[i].startswith("   ") or lineas[i].startswith("  - ")):
                t = lineas[i]
                if t.startswith("  - ") or t.startswith("   - "):
                    items[-1][1].append(t.strip()[2:])
                elif t.startswith("   ") and items:
                    items[-1][1].append(t.strip())
                else:
                    items.append([re.sub(r"^(\d+\. |- )", "", t), []])
                i += 1
            flo = []
            for texto, hijos in items:
                contenido = [Paragraph(inline(texto), E["p"])]
                if hijos:
                    contenido.append(ListFlowable([ListItem(Paragraph(inline(h), E["p"]), leftIndent=10)
                                                   for h in hijos], bulletType="bullet", start="–",
                                                  leftIndent=12, bulletFontName="DV"))
                flo.append(ListItem(contenido, leftIndent=14))
            story.append(ListFlowable(flo, bulletType="1" if numerada else "bullet",
                                      start=None if numerada else "•", leftIndent=14,
                                      bulletFontName="DV", bulletFontSize=9))
        else:
            parr = [l]
            i += 1
            while i < len(lineas) and lineas[i].strip() and not re.match(r"^(#|\||- |\d+\. |```|!\[)", lineas[i]):
                parr.append(lineas[i]); i += 1
            story.append(Paragraph(inline(" ".join(parr)), E["p"]))
    return story


def pie(canv, doc):
    canv.saveState()
    canv.setFont("DV", 7.5)
    canv.setFillColor(colors.grey)
    canv.drawString(1.8 * cm, 1.1 * cm, "IEE2913 · Grupo 9 · Conexiones y pruebas de laboratorio")
    canv.drawRightString(A4[0] - 1.8 * cm, 1.1 * cm, f"{doc.page}")
    canv.restoreState()


def main():
    entrada = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "conexiones_laboratorio.md")
    salida = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(entrada)[0] + ".pdf"
    doc = SimpleDocTemplate(salida, pagesize=A4, leftMargin=1.8 * cm, rightMargin=1.8 * cm,
                            topMargin=1.6 * cm, bottomMargin=1.8 * cm,
                            title="Conexiones y pruebas de laboratorio — pan-tilt G9", author="IEE2913 Grupo 9")
    story = convertir(open(entrada, encoding="utf-8").read(), os.path.dirname(os.path.abspath(entrada)),
                      A4[0] - 3.6 * cm)
    #* Un título nunca queda solo al pie: se une con lo que sigue (salvo tablas, que se parten).
    agrupada, k = [], 0
    while k < len(story):
        f = story[k]
        es_titulo = isinstance(f, Paragraph) and f.style.name in ("h2", "h3")
        if es_titulo and k + 1 < len(story) and not isinstance(story[k + 1], Table):
            agrupada.append(KeepTogether([f, story[k + 1]]))
            k += 2
        else:
            agrupada.append(f)
            k += 1
    doc.build(agrupada, onFirstPage=pie, onLaterPages=pie)
    print("PDF:", salida)


if __name__ == "__main__":
    main()
