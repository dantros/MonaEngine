"""
Equation extraction script for Memoria_Agustin_Matthey.md
Steps:
1. Detect equation blocks in markdown
2. Find their positions in the PDF using anchor text
3. Render those regions with pymupdf
4. OCR with pix2tex
5. Replace garbled blocks in markdown with $$ LaTeX $$
"""
import re
import sys
import io
import os
import pdfplumber
import fitz  # pymupdf

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

MD_PATH = r"g:\projects\mona_engine_\MonaEngine\Memorias\Memoria_Agustin_Matthey\Memoria_Agustin_Matthey.md"
PDF_PATH = r"g:\projects\mona_engine_\MonaEngine\Memorias\Memoria_Agustin_Matthey.pdf"
EQ_IMG_DIR = r"g:\projects\mona_engine_\MonaEngine\eq_imgs"
os.makedirs(EQ_IMG_DIR, exist_ok=True)

# Math characters that indicate equation content
MATH_CHARS = set('∂∑∫≈≤≥→←⇒·×√∞θφψαβγωΩπσλμΔδ∇⃗≠±∝∈∉⊂⊃⊆⊇∩∪∀∃¬⊕⊗˙')
MATH_CHARS.update(['⃗', '∆', '∂', '∇'])

# Words that indicate normal Spanish prose (not equations)
def has_spanish_word(line):
    """Returns True if line contains a recognizable Spanish word (≥5 letters)."""
    words = re.findall(r'[a-záéíóúüñA-ZÁÉÍÓÚÜÑ]{5,}', line)
    return len(words) >= 1

def is_math_line(line):
    """Returns True if line looks like part of an equation, not prose."""
    stripped = line.strip()
    if not stripped:
        return False
    if len(stripped) > 50:
        return False
    # Headings are not math
    if stripped.startswith('#'):
        return False
    # Figure captions are not math
    if stripped.startswith('*Figura') or stripped.startswith('!['):
        return False
    # Contains math chars
    has_math = any(c in MATH_CHARS for c in stripped)
    # Short line with math-like content (operators, letters, digits only)
    short_fragment = len(stripped) <= 8 and not has_spanish_word(stripped)
    # Explicit equation fragments: contains only math operators and simple identifiers
    looks_like_eq = re.match(r'^[a-zA-Z0-9 \+\-\*/\(\)\[\]\{\}\.,\^\_\|=<>∂∑∫≈≤≥→←·×√∞θφψαβγωΩπσλμΔδ∇⃗≠±˙⃗∆∇ ]+$', stripped)
    if has_math and not has_spanish_word(stripped):
        return True
    if short_fragment and looks_like_eq:
        return True
    return False

# ── Load markdown ──
with open(MD_PATH, encoding='utf-8') as f:
    md_lines = f.readlines()

print(f"Loaded markdown: {len(md_lines)} lines")

# ── Detect equation blocks ──
# An equation block: 2+ consecutive equation lines (blank lines within count)
# We expand blocks to include surrounding blank lines

in_block = False
blocks = []
current_block_start = None
last_math_line = None

for i, line in enumerate(md_lines):
    if is_math_line(line):
        if not in_block:
            # Start block, looking back for blank lines
            start = i
            while start > 0 and md_lines[start-1].strip() == '':
                start -= 1
            current_block_start = start
            in_block = True
        last_math_line = i
    elif line.strip() == '' and in_block:
        # Blank line - keep block open
        pass
    else:
        if in_block:
            # End block at last math line + trailing blanks
            end = last_math_line
            while end + 1 < len(md_lines) and md_lines[end+1].strip() == '':
                end += 1
            if end - current_block_start >= 1:  # at least 2 lines span
                blocks.append((current_block_start, end))
            in_block = False

# Merge adjacent/overlapping blocks
merged = []
for b in blocks:
    if merged and b[0] <= merged[-1][1] + 3:
        merged[-1] = (merged[-1][0], max(merged[-1][1], b[1]))
    else:
        merged.append(list(b))
blocks = [tuple(b) for b in merged]

print(f"\nDetected {len(blocks)} equation blocks:")
for i, (s, e) in enumerate(blocks):
    print(f"\n  Block {i:02d} (lines {s+1}–{e+1}):")
    for li in range(s, min(e+1, s+8)):
        print(f"    {li+1:4d}: {md_lines[li].rstrip()}")
    if e - s >= 8:
        print(f"    ... ({e-s-7} more lines)")
