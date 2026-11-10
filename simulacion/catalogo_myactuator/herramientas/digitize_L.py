"""Digitaliza las graficas de curva caracteristica de la serie L (imagen raster, sin tabla).
Ejes: x = velocidad [rpm]; y: par [mN.m], corriente DC [A], eficiencia [fraccion], potencia [W]."""
import sys, json, numpy as np
from PIL import Image
REF = {"I": (189, 72, 66), "Eff": (129, 171, 86), "Pout": (34, 34, 34), "T": (79, 81, 132)}
BG = [(255, 255, 255), (225, 225, 225), (200, 200, 200), (240, 220, 220)]
def load(f):
    return np.asarray(Image.open(f).convert("RGB")).astype(float)
def classify(a):
    R, G, B = a[..., 0], a[..., 1], a[..., 2]
    lum = a.mean(-1); mx = a.max(-1); mn = a.min(-1)
    vis = lum < 215                                   # descarta fondo y rejilla clara
    return {
        "I":    vis & (R - G > 30) & (R - B > 20),
        "Eff":  vis & (G - R >= 0) & (G - B > 18),
        "T":    vis & (B - R > 18) & (B - G > 15),
        "Pout": (mx < 165) & (mx - mn < 28),
    }
def frame(a):
    R, G, B = a[..., 0], a[..., 1], a[..., 2]
    gray = (abs(R-G) < 18) & (abs(G-B) < 18) & (R < 235) & (R > 120)
    rows = gray.sum(1); H, W = gray.shape
    rr = [i for i in range(H) if rows[i] > 0.45*W]
    return rr[0], rr[-1]
def axes_x(a, top, bot):
    """x0 = linea vertical rojiza (eje de corriente, rpm=0); x1 = linea vertical verdosa (eje de eficiencia, rpm max)."""
    seg = a[top+3:bot-3]; col = seg.mean(0); lum = col.mean(1); W = a.shape[1]
    dark = np.array([((lum[j-2]+lum[j+2])/2 - lum[j]) if 2 <= j < W-2 else 0 for j in range(W)])
    R, G, B = col[:, 0], col[:, 1], col[:, 2]
    red = [j for j in range(W//2) if dark[j] > 18 and R[j]-G[j] > 10]
    grn = [j for j in range(W//2, W) if dark[j] > 25 and G[j]-R[j] > 8 and G[j]-B[j] > 8]
    x0 = max(red, key=lambda j: dark[j]); x1 = max(grn, key=lambda j: dark[j])
    return x0, x1
def run(f, xmax, ymax):
    a = load(f); H, W, _ = a.shape
    top, bot = frame(a)
    cls = classify(a)
    x0, x1 = axes_x(a, top, bot)
    info = dict(top=top, bot=bot, x0=x0, x1=x1)
    curves = {}
    for name in REF:
        m = cls[name].copy()
        m[:, :x0+2] = False; m[:, x1-1:] = False; m[:top+2] = False; m[bot-2:] = False
        cols = {j: np.where(m[:, j])[0] for j in range(x0+2, x1-1)}
        # arranque: Pout y Eff salen del origen; T e I de la mediana de las primeras columnas con datos
        if name in ("Pout", "Eff"): prev = float(bot - 3); slope = -1.0
        else:
            first = [y for j in range(x0+2, x0+12) for y in cols[j]]
            prev = float(np.median(first)) if first else None
        pts = []; gap = 0; hist = []
        slope = -1.0 if name in ("Pout", "Eff") else 0.0
        for j in range(x0+2, x1-1):
            ys = cols[j]
            if len(ys) == 0 or prev is None:
                gap += 1; continue
            pred = prev + slope*(gap+1)                       # prediccion por pendiente (cruza otras curvas)
            y = float(ys[np.argmin(abs(ys - pred))])
            if abs(y - pred) > 4 + 2*gap and name != "Eff":   # salto: pixel de otra curva/texto
                gap += 1; continue
            hist.append((j, y)); hist = hist[-8:]
            if len(hist) >= 4:
                hj = np.array(hist); slope = float(np.polyfit(hj[:, 0], hj[:, 1], 1)[0])
            prev = y; gap = 0
            rpm = (j - x0) / (x1 - x0) * xmax
            pts.append((rpm, (bot - y) / (bot - top) * ymax[name], 1))
        curves[name] = pts
    return info, curves
if __name__ == "__main__":
    f, xmax, yT, yI, yE, yP, out = sys.argv[1:8]
    info, curves = run(f, float(xmax), {"T": float(yT), "I": float(yI), "Eff": float(yE), "Pout": float(yP)})
    print(info, {k: len(v) for k, v in curves.items()})
    json.dump({"info": info, "curves": curves}, open(out, "w"))
