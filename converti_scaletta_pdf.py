from pathlib import Path
import re

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
)


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "scaletta_presentazione_4_relatori.md"
OUTPUT = ROOT / "scaletta_presentazione_4_relatori.pdf"


def register_fonts():
    candidates = [
        ("C:/Windows/Fonts/arial.ttf", "C:/Windows/Fonts/arialbd.ttf"),
        ("C:/Windows/Fonts/calibri.ttf", "C:/Windows/Fonts/calibrib.ttf"),
    ]
    for regular, bold in candidates:
        if Path(regular).exists() and Path(bold).exists():
            pdfmetrics.registerFont(TTFont("DocRegular", regular))
            pdfmetrics.registerFont(TTFont("DocBold", bold))
            return
    raise FileNotFoundError("Nessun font Unicode compatibile trovato")


def inline_markup(value):
    value = value.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    value = re.sub(r"`([^`]+)`", r'<font name="Courier">\1</font>', value)
    value = re.sub(r"\*\*([^*]+)\*\*", r"<b>\1</b>", value)
    return value


def footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor("#CBD5E1"))
    canvas.line(18 * mm, 14 * mm, 192 * mm, 14 * mm)
    canvas.setFont("DocRegular", 8)
    canvas.setFillColor(colors.HexColor("#64748B"))
    canvas.drawString(18 * mm, 9 * mm, "Map Construction in PDDL+ — Copione d'esame")
    canvas.drawRightString(192 * mm, 9 * mm, f"Pagina {doc.page}")
    canvas.restoreState()


register_fonts()
styles = getSampleStyleSheet()
styles.add(ParagraphStyle(
    "DocTitle", parent=styles["Title"], fontName="DocBold", fontSize=22,
    leading=27, textColor=colors.HexColor("#172554"), alignment=TA_CENTER,
    spaceAfter=12,
))
styles.add(ParagraphStyle(
    "H2x", parent=styles["Heading2"], fontName="DocBold", fontSize=16,
    leading=20, textColor=colors.HexColor("#1D4ED8"), spaceBefore=12, spaceAfter=8,
))
styles.add(ParagraphStyle(
    "H3x", parent=styles["Heading3"], fontName="DocBold", fontSize=12.5,
    leading=16, textColor=colors.HexColor("#0F766E"), spaceBefore=9, spaceAfter=5,
))
styles.add(ParagraphStyle(
    "BodyX", parent=styles["BodyText"], fontName="DocRegular", fontSize=10.3,
    leading=15, textColor=colors.HexColor("#1E293B"), spaceAfter=7,
))
styles.add(ParagraphStyle(
    "BulletX", parent=styles["BodyText"], fontName="DocRegular", fontSize=10.2,
    leading=14.5, leftIndent=7 * mm, firstLineIndent=-4 * mm,
    textColor=colors.HexColor("#1E293B"), spaceAfter=5,
))
styles.add(ParagraphStyle(
    "QuoteX", parent=styles["BodyText"], fontName="DocRegular", fontSize=10,
    leading=15, leftIndent=7 * mm, rightIndent=5 * mm,
    borderColor=colors.HexColor("#93C5FD"), borderWidth=1,
    borderPadding=7, backColor=colors.HexColor("#EFF6FF"),
    textColor=colors.HexColor("#1E3A5F"), spaceAfter=8,
))

story = []
lines = SOURCE.read_text(encoding="utf-8").splitlines()
for raw in lines:
    line = raw.strip()
    if not line:
        story.append(Spacer(1, 2.5 * mm))
        continue
    if line.startswith("# "):
        story.append(Paragraph(inline_markup(line[2:]), styles["DocTitle"]))
    elif line.startswith("## "):
        if story:
            story.append(PageBreak())
        story.append(Paragraph(inline_markup(line[3:]), styles["H2x"]))
    elif line.startswith("### "):
        story.append(Paragraph(inline_markup(line[4:]), styles["H3x"]))
    elif line.startswith("- "):
        story.append(Paragraph("• " + inline_markup(line[2:]), styles["BulletX"]))
    elif line.startswith("«") and line.endswith("»"):
        story.append(Paragraph(inline_markup(line), styles["QuoteX"]))
    else:
        story.append(Paragraph(inline_markup(line), styles["BodyX"]))

doc = SimpleDocTemplate(
    str(OUTPUT), pagesize=A4,
    rightMargin=18 * mm, leftMargin=18 * mm,
    topMargin=17 * mm, bottomMargin=20 * mm,
    title="Copione presentazione Map Construction in PDDL+",
    author="Chiara, Elisa, Emanuele, Pierluigi",
)
doc.build(story, onFirstPage=footer, onLaterPages=footer)
print(f"Creato: {OUTPUT}")
