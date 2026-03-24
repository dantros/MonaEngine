#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

MD_PATH = r'g:/projects/mona_engine_/MonaEngine/Memorias/Memoria_Agustin_Matthey/Memoria_Agustin_Matthey.md'

with open(MD_PATH, 'r', encoding='utf-8') as f:
    text = f.read()

def rep(old, new):
    global text
    if old not in text:
        print(f'WARNING not found: {repr(old[:80])}')
    else:
        text = text.replace(old, new)
        print(f'OK: replaced {repr(old[:60])}')

HAT = '\u02c6'   # ˆ  modifier letter circumflex (U+02C6, ord 710)

# ── Patch 1: Eq 4.2 (eePos decomposition) ────────────────────────────────────
rep(
'constantes. Notar primero que eePos(\u03b8) puede descomponerse de la siguiente manera:\n'
'eeP\n'
'os(\u03b8\n'
') = M\n'
+ HAT + '\n'
'AT\u03b8 kR\u03b8 kS\u03b8\n'
'kM\n'
+ HAT + '\n'
'B\n'
+ HAT + '\n'
'b (4.2)',
'constantes. Notar primero que $\\overrightarrow{eePos}(\\vec{\\theta})$ puede descomponerse de la siguiente manera:\n'
'\n'
'$$\\overrightarrow{eePos}(\\vec{\\theta}) = \\hat{M}_A T_{\\theta_k} R_{\\theta_k} S_{\\theta_k} \\hat{M}_B \\hat{\\vec{b}} \\tag{4.2}$$'
)

# ── Patch 2: Eq 4.3 (MA, b definitions) ──────────────────────────────────────
rep(
'constantes, quedando\n'
'MA\n'
'= M\n'
+ HAT + '\n'
'AT\u03b8\n'
'k\n'
'y\n'
'b =\n'
'S\u03b8\n'
'kM\n'
+ HAT + '\n'
'B\n'
+ HAT + '\n'
'b = {b ,b ,b ,b }:\n'
'0 1 2 3\n'
'eeP\n'
'os(\u03b8\n'
') =\n'
'MAR\u03b8\n'
'k\n'
'b (4.3)',
'constantes, quedando $M_A = \\hat{M}_A T_{\\theta_k}$ y $\\vec{b} = S_{\\theta_k} \\hat{M}_B \\hat{\\vec{b}} = \\{b_0, b_1, b_2, b_3\\}$:\n'
'\n'
'$$\\overrightarrow{eePos}(\\vec{\\theta}) = M_A R_{\\theta_k} \\vec{b} \\tag{4.3}$$'
)

# ── Patch 3: theta = gamma initialization ─────────────────────────────────────
rep(
'se escoge entonces inicializar el vector de variables como\n'
'sigue:\n'
'\u03b8 = \u20d7\u03b3',
'se escoge entonces inicializar el vector de variables como sigue:\n'
'\n'
'$$\\vec{\\theta} = \\vec{\\gamma}$$'
)

with open(MD_PATH, 'w', encoding='utf-8') as f:
    f.write(text)

print('\nPatch done.')
