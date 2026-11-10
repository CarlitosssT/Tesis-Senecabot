"""OCR de la tabla de datos (lado derecho) de una imagen de curva caracteristica MyActuator.
Prueba varios recortes/umbrales y se queda con el que da mas filas fisicamente validas."""
import re, subprocess, sys, math, os, tempfile
from PIL import Image, ImageOps
def valid(r, k=1.0):
    T, N, Po, U, I, Pi, E = r
    return abs(T*k*N*2*math.pi/60 - Po) <= max(0.04*Po, 0.2) and abs(U*I - Pi) <= max(0.02*Pi, 0.2) and 15 <= U <= 60
def ocr(im, x0, y1, thr, scale=4):
    W, H = im.size
    c = im.crop((int(x0*W), int(0.05*H), W, int(y1*H)))
    c = c.resize((c.width*scale, c.height*scale), Image.LANCZOS)
    c = ImageOps.autocontrast(c).point(lambda v: 255 if v > thr else 0)
    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as t: c.save(t.name)
    txt = subprocess.run(["tesseract", t.name, "-", "--psm", "6", "-c", "tessedit_char_whitelist=0123456789."],
                         capture_output=True, text=True).stdout
    os.unlink(t.name)
    rows = []
    for line in txt.splitlines():
        nums = re.findall(r"\d+\.\d+|\d+", line)
        if len(nums) == 7: rows.append([float(x) for x in nums])
    return rows
img, out = sys.argv[1], sys.argv[2]
im = Image.open(img).convert("L")
best = None
for x0 in (0.55, 0.565, 0.58, 0.595, 0.61):
    for y1 in (0.70, 0.80):
        for thr in (150, 175, 200):
            rows = ocr(im, x0, y1, thr)
            nv = sum(valid(r) for r in rows)
            if best is None or (nv, len(rows)) > (best[0], len(best[1])): best = (nv, rows, (x0, y1, thr))
nv, rows, cfg = best
with open(out + ".txt", "w") as f:
    for r in rows: f.write(repr(r) + ("" if valid(r) else "   # INVALIDA") + "\n")
print(f"{len(rows)} filas, {nv} validas, recorte {cfg}")
