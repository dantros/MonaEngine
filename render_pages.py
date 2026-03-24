"""Render key PDF pages to identify equation regions."""
import sys, io
import fitz
import pdfplumber

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PDF_PATH = r"g:\projects\mona_engine_\MonaEngine\Memorias\Memoria_Agustin_Matthey.pdf"
OUT_DIR = r"g:\projects\mona_engine_\MonaEngine\eq_imgs"

import os
os.makedirs(OUT_DIR, exist_ok=True)

# Find pages containing equations by searching for math-heavy content
SEARCH_TERMS = [
    "función inversa, tal que",    # block 10 - IK equation
    "una función escalar, multivariable",  # block 12-14 - gradient
    "inversión del Jacobiano",      # blocks 15-22 - Jacobian
    "applica el descenso de gradiente descrito",  # block 22 - F function
    "dividimos M, la ecuación queda",  # blocks 4-6 - Ch2.1 equation
    "cuaterniones unitarios",           # block 7 - quaternion
    "construccíon de una función total",  # Ch4 gradient
]

doc = fitz.open(PDF_PATH)

print(f"PDF has {len(doc)} pages\n")

with pdfplumber.open(PDF_PATH) as pdf:
    for page_num, page in enumerate(pdf.pages):
        text = page.extract_text() or ''
        for term in SEARCH_TERMS:
            # try partial match
            key = term[:25]
            if key in text:
                print(f"Page {page_num+1}: found '{key}'")
                # Render this page at 3x
                fpage = doc[page_num]
                mat = fitz.Matrix(3, 3)
                pix = fpage.get_pixmap(matrix=mat)
                out_path = os.path.join(OUT_DIR, f"page_{page_num+1:03d}.png")
                pix.save(out_path)
                print(f"  -> saved {out_path}")
                break
